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

class InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('inference_node')

        self.declare_parameter('model_name', 'model.pt')
        self.declare_parameter('interval_ms', 100)

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value

        self.bridge = CvBridge()
        self.device = torch.device('cuda')
        self.latest_image = None

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
        self.timer = self.create_timer(interval_ms / 1000.0, self.timer_callback)

    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        resized = cv2.resize(image, (64, 48))
        normalized = resized.astype(np.float32) / 255.0
        tensor = torch.from_numpy(normalized).permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.device)

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def timer_callback(self) -> None:
        if self.latest_image is None:   return

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')
        input_tensor = self.preprocess_image(cv_image)

        with torch.no_grad():
            output = self.model(input_tensor)

        path_msg = self.create_path_from_output(output, self.latest_image.header)
        self.pub.publish(path_msg)

    def create_path_from_output(self, output: torch.Tensor, header) -> Path:
        path_msg = Path()
        path_msg.header = header
        waypoints = output.cpu().numpy().reshape(-1, 2)

        path_msg.poses = [PoseStamped(header=header, pose=Pose(position=Point(x=float(x), y=float(y)))) for x, y in waypoints]

        return path_msg

def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
