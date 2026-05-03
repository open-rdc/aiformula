import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, UInt8
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
from typing import Tuple
from .zed_sdk import ZedSdk

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
        self.declare_parameter('use_place_recognition', True)
        self.declare_parameter('placenet_model_name', 'placenet.pt')
        self.declare_parameter('topomap_dir_name', 'topomap')
        self.declare_parameter('placenet_delta', 10.0)
        self.declare_parameter('placenet_window_lower', -1)
        self.declare_parameter('placenet_window_upper', 2)

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        self.command = int(self.get_parameter('default_command').value)
        self.use_place_recognition = bool(self.get_parameter('use_place_recognition').value)
        placenet_model_name = self.get_parameter('placenet_model_name').value
        topomap_dir_name = self.get_parameter('topomap_dir_name').value
        self.placenet_delta = float(self.get_parameter('placenet_delta').value)
        self.placenet_window_lower = int(self.get_parameter('placenet_window_lower').value)
        self.placenet_window_upper = int(self.get_parameter('placenet_window_upper').value)

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.zed = None

        self.cv_image = None

        self.sim_flag = False

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        yolop_weight_path = FilePath(package_share_directory) / 'weights' / 'yolopv2.pt'
        placenet_weight_path = FilePath(package_share_directory) / 'weights' / placenet_model_name
        topomap_path = FilePath(package_share_directory) / 'config' / topomap_dir_name / 'topomap.yaml'

        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)
        self.place_recognition = None
        if self.use_place_recognition:
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
        self.zed = ZedSdk(self, self.sim_flag)
        self.create_subscription(UInt8, '/command', self.command_callback, qos_profile_system_default)

        self.pub_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
        self.pub_pointcloud = self.create_publisher(PointCloud2, '/zed/zed_node/pointcloud', qos_profile_sensor_data)
        self.pub_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)
        self.get_logger().info('Debug mode enabled: publishing preprocessed images to e2e_planner/debug_image')

        self.torch_cb_group = ReentrantCallbackGroup()
        self.zed_cb_group = ReentrantCallbackGroup()

        self.torch_timer = self.create_timer(
            interval_ms / 1000.0,
            self.torch_callback,
            callback_group=self.torch_cb_group,
        )
        self.zed_timer = self.create_timer(
            10.0 / 1000.0,
            self.zed_sensor_callback,
            callback_group=self.zed_cb_group,
        )

    def command_callback(self, msg: UInt8) -> None:
        self.command = int(msg.data)

    def preprocess_command(self, command: int | None = None) -> torch.Tensor:
        command_tensor = torch.zeros((1, 4), device=self.device, dtype=torch.float32)
        command_idx = self.command if command is None else int(command)
        command_idx = min(max(command_idx, 0), 3)
        command_tensor[0, command_idx] = 1.0
        return command_tensor

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
        if self.sim_flag:
            mask = ((image[:, :, 2] > 200) & (image[:, :, 0] < 50) & (image[:, :, 1] < 50)).astype(np.uint8)
        else:
            bgr_image = self._to_bgr(image)
            mask = self.yolop_processor.process_image(bgr_image)

        mask_normalized = lane_mask_to_tensor_array(mask)
        tensor = torch.from_numpy(mask_normalized).unsqueeze(0).unsqueeze(0)
        return tensor.to(self.device), mask_normalized

    def torch_callback(self) -> None:
        if self.cv_image is None or self.model is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        input_tensor, mask = self.preprocess_image(self.cv_image)
        command = self.recognize_command(self.cv_image)
        command_tensor = self.preprocess_command(command)

        debug_image = overlay_lane_mask(self._to_bgr(self.cv_image), mask)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = header
        self.pub_debug_image.publish(debug_msg)

        with torch.no_grad():
            output = self.model(input_tensor, command_tensor)

        output_normalized = output.cpu().numpy().flatten()
        output_denormalized = denormalize_waypoints(output_normalized)
        output_denormalized_tensor = torch.from_numpy(output_denormalized).unsqueeze(0)

        path_raw_msg = self.create_path_from_output(output_denormalized_tensor, header)
        self.pub_raw.publish(path_raw_msg)

        path_smooth_msg = self.apply_bspline_smoothing(output_denormalized_tensor, header)
        self.pub.publish(path_smooth_msg)

    def zed_sensor_callback(self) -> None:
        if self.zed is None or not self.zed.grab():
            return

        self.cv_image = self.zed.get_image()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        pointcloud_msg = self.zed.get_pointcloud(header)
        if pointcloud_msg is None:
            pointcloud_msg = PointCloud2()
            pointcloud_msg.header = header
        self.pub_pointcloud.publish(pointcloud_msg)

    def apply_bspline_smoothing(self, output: torch.Tensor, header) -> Path:
        waypoints = output.cpu().numpy().reshape(-1, 2)
        x = waypoints[:, 0]
        y = waypoints[:, 1]

        # s: smoothing factor（値が大きいほど滑らか、0だと補間）
        # k: スプラインの次数（3次）
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
