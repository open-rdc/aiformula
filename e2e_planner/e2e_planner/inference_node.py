import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev
from typing import Optional

try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False

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
        self.declare_parameter('sdk_flag', False)

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        self.sdk_flag_ = self.get_parameter('sdk_flag').value

        self.bridge = CvBridge()
        self.device = torch.device('cuda')
        self.latest_image = None
        self.zed_camera: Optional[sl.Camera] = None
        self.zed_image: Optional[sl.Mat] = None
        self.zed_runtime_params: Optional[sl.RuntimeParameters] = None

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        if self.sdk_flag_:
            if not ZED_SDK_AVAILABLE:
                self.get_logger().error('ZED SDK not available. Install pyzed package.')
                raise RuntimeError('ZED SDK not available')
            self._initialize_zed_camera()
        else:
            self.sub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)

        self.pub_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
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
            return self.zed_image.get_data()
        return None

    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        image = image[:, 40:440]
        resized = cv2.resize(image, (64, 48))
        normalized = resized.astype(np.float32) / 255.0
        tensor = torch.from_numpy(normalized).permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.device)

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def timer_callback(self) -> None:
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
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')
            header = self.latest_image.header

        input_tensor = self.preprocess_image(cv_image)

        with torch.no_grad():
            output = self.model(input_tensor)

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

        # s: smoothing factor（値が大きいほど滑らか、0だと補間）
        # k: スプラインの次数（3次）
        tck, u = splprep([x, y], s=0.1, k=3)
        u_new = np.linspace(0, 1, 30)
        x_smooth, y_smooth = splev(u_new, tck)

        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = 'base_link'

        path_msg.poses = [PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x_smooth[i]), y=float(y_smooth[i])))) for i in range(len(x_smooth))]

        return path_msg

    def create_path_from_output(self, output: torch.Tensor, header) -> Path:
        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = 'base_link'
        waypoints = output.cpu().numpy().reshape(-1, 2)

        path_msg.poses.append(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses = [PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x), y=float(y)))) for x, y in waypoints]

        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
