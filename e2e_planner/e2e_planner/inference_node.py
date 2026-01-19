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

def denormalize_waypoints(normalized: np.ndarray) -> np.ndarray:
    denormalized = normalized.copy()
    denormalized[0::2] = (normalized[0::2] + 1.0) * 5.0
    denormalized[1::2] = (normalized[1::2] + 1.0) * 3.0 - 3.0
    return denormalized

class InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('inference_node')
        self.declare_parameter('model_name', 'e2e_model(2).pt')
        self.declare_parameter('interval_ms', 100)

        model_name = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value
        
        pkg_share = get_package_share_directory('e2e_planner')
        model_path = os.path.join(pkg_share, 'weights', model_name)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        self.pub_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
        self.get_logger().info('InferenceNode initialized. Waiting for images...')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        self.model = torch.jit.load(model_path, map_location=self.device)
        self.model.to(self.device)
        self.model.eval()
        self.latest_image = None
        
        self.timer = self.create_timer(interval_ms / 1000.0, self.timer_callback)

    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        image = image[:, 40:440]
        resized = cv2.resize(image, (64, 48))
        normalized = resized.astype(np.float32) / 255.0
        if len(normalized.shape) == 2:
            normalized = np.expand_dims(normalized, axis=2)
        tensor = torch.from_numpy(normalized).permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.device)

    def image_callback(self, msg: Image) -> None:
        if self.latest_image is None:
            self.get_logger().info('First image received!')
        self.latest_image = msg

    def timer_callback(self) -> None:
        if self.latest_image is None:   return

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_image = np.expand_dims(gray_image, axis=2)
        input_tensor = self.preprocess_image(gray_image)

        with torch.no_grad():
            output = self.model(input_tensor)

        output_normalized = output.cpu().numpy().flatten()
        output_denormalized = denormalize_waypoints(output_normalized)
        output_denormalized_tensor = torch.from_numpy(output_denormalized).unsqueeze(0)

        path_raw_msg = self.create_path_from_output(output_denormalized_tensor, self.latest_image.header)
        self.pub_raw.publish(path_raw_msg)

        path_smooth_msg = self.apply_bspline_smoothing(output_denormalized_tensor, self.latest_image.header)
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
