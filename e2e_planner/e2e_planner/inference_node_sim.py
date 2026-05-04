import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from cv_bridge import CvBridge
import cv2
import torch
from torchvision import transforms
import numpy as np
import os
from pathlib import Path as FilePath
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev
from typing import Optional, Tuple

try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

from e2e_planner.placenav.place_recognition import PlaceRecognition
from util.yolop_processor import YOLOPv2Processor
from util.preprocessing import center_square_crop, lane_mask_to_tensor_array, overlay_lane_mask


def denormalize_waypoints(normalized: np.ndarray) -> np.ndarray:
    denormalized = normalized.copy()
    denormalized[0::2] = (normalized[0::2] + 1.0) * 5.0
    denormalized[1::2] = (normalized[1::2] + 1.0) * 3.0 - 3.0
    return denormalized


class InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('inference_node')

        self.declare_parameter('model_name', 'model.pt')
        self.declare_parameter('interval_ms', 100)
        self.declare_parameter('default_command', 1)
        self.declare_parameter('use_place_recognition', False)
        self.declare_parameter('placenet_model_name', 'placenet.pt')
        self.declare_parameter('topomap_dir_name', 'topomap')
        self.declare_parameter('placenet_delta', 10.0)
        self.declare_parameter('placenet_window_lower', -1)
        self.declare_parameter('placenet_window_upper', 2)
        self.declare_parameter('sdk_flag', False)
        self.declare_parameter('debug_mode', True)

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        self.command = int(self.get_parameter('default_command').value)
        self.use_place_recognition = bool(self.get_parameter('use_place_recognition').value)
        placenet_model_name = self.get_parameter('placenet_model_name').value
        topomap_dir_name = self.get_parameter('topomap_dir_name').value
        self.placenet_delta = float(self.get_parameter('placenet_delta').value)
        self.placenet_window_lower = int(self.get_parameter('placenet_window_lower').value)
        self.placenet_window_upper = int(self.get_parameter('placenet_window_upper').value)
        self.sdk_flag_ = self.get_parameter('sdk_flag').value
        self.debug_mode_ = self.get_parameter('debug_mode').value

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.latest_image = None
        self.zed_camera: Optional[sl.Camera] = None
        self.zed_image: Optional[sl.Mat] = None
        self.zed_runtime_params: Optional[sl.RuntimeParameters] = None

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        yolop_weight_path = FilePath(package_share_directory) / 'weights' / 'yolopv2.pt'
        placenet_weight_path = FilePath(package_share_directory) / 'weights' / placenet_model_name
        topomap_path = FilePath(package_share_directory) / 'config' / topomap_dir_name / 'topomap.yaml'

        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
            self.model_uses_command = self._model_uses_command(self.model)
            if not self.model_uses_command:
                self.get_logger().warn(
                    'Loaded model accepts only image input; navigation command will be ignored.'
                )
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None
            self.model_uses_command = False

        self.yolop_processor = None
        if yolop_weight_path.exists():
            self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)
            self.get_logger().info('Warming up YOLOP (JIT compile)...')
            dummy = np.zeros((300, 480, 3), dtype=np.uint8)
            self.yolop_processor.process_image(dummy)
            self.get_logger().info('YOLOP warmup done.')
        else:
            self.get_logger().warn(f'YOLOPv2 model not found: {yolop_weight_path}')

        self.place_recognition = None
        if self.use_place_recognition:
            if not placenet_weight_path.exists():
                self.get_logger().warn(f'Place recognition model not found: {placenet_weight_path}')
            elif not topomap_path.exists():
                self.get_logger().warn(f'Topomap not found: {topomap_path}')
            else:
                self.place_recognition = PlaceRecognition(
                    placenet_weight_path,
                    topomap_path,
                    device=self.device,
                    delta=self.placenet_delta,
                    window_lower=self.placenet_window_lower,
                    window_upper=self.placenet_window_upper,
                )
        self.placenet_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((85, 85), antialias=True),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        self.create_subscription(UInt8, '/command', self.command_callback, qos_profile_system_default)

        if self.sdk_flag_:
            if not ZED_SDK_AVAILABLE:
                self.get_logger().error('ZED SDK not available. Install pyzed package.')
                raise RuntimeError('ZED SDK not available')
            self._initialize_zed_camera()
        else:
            self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        self.pub_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)

        if self.debug_mode_:
            self.pub_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)
            self.get_logger().info('Debug mode enabled: publishing preprocessed images to e2e_planner/debug_image')

        self.timer = self.create_timer(interval_ms / 1000.0, self.timer_callback)

    def _initialize_zed_camera(self) -> None:
        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30

        err = self.zed_camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED camera: {err}')
            raise RuntimeError(f'Failed to open ZED camera: {err}')

        self.zed_image = sl.Mat()
        self.zed_runtime_params = sl.RuntimeParameters()
        self.get_logger().info('ZED camera initialized successfully')

    def _capture_image_from_zed(self) -> Optional[np.ndarray]:
        if self.zed_camera.grab(self.zed_runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed_camera.retrieve_image(self.zed_image, sl.VIEW.LEFT)
            image = self.zed_image.get_data()
            height, width = image.shape[:2]
            resized_image = cv2.resize(image, (width // 2, height // 2))
            return resized_image
        return None

    def command_callback(self, msg: UInt8) -> None:
        self.command = int(msg.data)

    def preprocess_command(self, command: int | None = None) -> torch.Tensor:
        command_tensor = torch.zeros((1, 4), device=self.device, dtype=torch.float32)
        command_idx = self.command if command is None else int(command)
        command_idx = min(max(command_idx, 0), 3)
        command_tensor[0, command_idx] = 1.0
        return command_tensor

    def _model_uses_command(self, model) -> bool:
        try:
            schema = str(model.forward.schema)
        except RuntimeError:
            return True
        return 'Tensor cmd' in schema or 'Tensor command' in schema

    def run_model(self, input_tensor: torch.Tensor, command_tensor: torch.Tensor) -> torch.Tensor:
        if self.model_uses_command:
            return self.model(input_tensor, command_tensor)
        return self.model(input_tensor)

    def preprocess_placenet_image(self, image: np.ndarray) -> torch.Tensor:
        bgr_image = self._to_bgr(image)
        cropped_image = center_square_crop(bgr_image)
        rgb_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
        placenet_image_tensor = self.placenet_transform(rgb_image).unsqueeze(0)
        return placenet_image_tensor.to(self.device, dtype=torch.float32)

    def recognize_command(self, image: np.ndarray) -> int:
        if self.place_recognition is None:
            return self.command

        placenet_image_tensor = self.preprocess_placenet_image(image)
        command, idx = self.place_recognition.get_recognition(placenet_image_tensor)
        self.command = int(command)
        self.get_logger().info(f'place recognition: command={command}, idx={idx}')
        return int(command)

    def _to_bgr(self, image: np.ndarray) -> np.ndarray:
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def preprocess_image(self, image: np.ndarray) -> Tuple[torch.Tensor, np.ndarray]:
        bgr_image = self._to_bgr(image)
        if self.yolop_processor is None:
            raise RuntimeError('YOLOPv2 processor is not available.')
        mask = self.yolop_processor.process_image(bgr_image)

        mask_normalized = lane_mask_to_tensor_array(mask)
        tensor = torch.from_numpy(mask_normalized).unsqueeze(0).unsqueeze(0)
        return tensor.to(self.device), mask_normalized

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def timer_callback(self) -> None:
        if self.model is None or self.yolop_processor is None:
            return

        if self.sdk_flag_:
            cv_image = self._capture_image_from_zed()
            if cv_image is None:
                return
            from std_msgs.msg import Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'
        else:
            if self.latest_image is None:
                return
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            header = self.latest_image.header

        input_tensor, mask = self.preprocess_image(cv_image)
        command = self.recognize_command(cv_image)
        command_tensor = self.preprocess_command(command)
        if self.debug_mode_:
            debug_image = overlay_lane_mask(self._to_bgr(cv_image), mask)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_image.publish(debug_msg)

        with torch.no_grad():
            output = self.run_model(input_tensor, command_tensor)

        output_normalized = output.cpu().numpy().flatten()
        output_denormalized = denormalize_waypoints(output_normalized)
        output_denormalized_tensor = torch.from_numpy(output_denormalized).unsqueeze(0)

        path_raw_msg = self.create_path_from_output(output_denormalized_tensor, header)
        self.pub_raw.publish(path_raw_msg)

        path_smooth_msg = self.apply_bspline_smoothing(output_denormalized_tensor, header)
        self.pub.publish(path_smooth_msg)

    def apply_bspline_smoothing(self, output: torch.Tensor, header) -> Path:
        waypoints = output.cpu().numpy().reshape(-1, 2)
        x = waypoints[:, 0]
        y = waypoints[:, 1]

        tck, u = splprep([x, y], s=0.1, k=3)
        u_new = np.linspace(0, 1, 30)
        x_smooth, y_smooth = splev(u_new, tck)

        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = 'base_link'

        path_msg.poses.append(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses.extend(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x_smooth[i]), y=float(y_smooth[i])))) for i in range(len(x_smooth)))

        return path_msg

    def create_path_from_output(self, output: torch.Tensor, header) -> Path:
        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = 'base_link'
        waypoints = output.cpu().numpy().reshape(-1, 2)

        path_msg.poses.append(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses.extend(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x), y=float(y)))) for x, y in waypoints)

        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
