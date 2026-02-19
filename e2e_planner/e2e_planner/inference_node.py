import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
from pathlib import Path as FilePath
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev
from typing import Tuple
from .zed_sdk import ZedSdk

from util.yolop_processor import YOLOPv2Processor

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

        model_path = self.get_parameter('model_name').value
        interval_ms = self.get_parameter('interval_ms').value

        self.bridge = CvBridge()
        self.device = torch.device('cuda')
        self.zed = None

        self.sim_flag = False

        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', model_path)
        yolop_weight_path = FilePath(package_share_directory) / 'weights' / 'yolopv2.pt'

        if os.path.exists(weight_path):
            self.model = torch.jit.load(weight_path, map_location=self.device)
            self.model.eval()
        else:
            self.get_logger().warn(f'Model file not found: {weight_path}')
            self.model = None

        self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)
        self.zed = ZedSdk(self, self.sim_flag)

        self.pub_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.pub = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
        self.pub_pointcloud = self.create_publisher(PointCloud2, '/zed/zed_node/pointcloud', qos_profile_sensor_data)
        self.pub_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)
        self.get_logger().info('Debug mode enabled: publishing preprocessed images to e2e_planner/debug_image')

        self.timer = self.create_timer(interval_ms / 1000.0, self.timer_callback)
        self.pcl_timer = self.create_timer(1.0 / 50.0, self.pcl_publisher_callback)

    def preprocess_image(self, image: np.ndarray) -> Tuple[torch.Tensor, np.ndarray]:
        if self.sim_flag:
            mask = ((image[:, :, 2] > 200) & (image[:, :, 0] < 50) & (image[:, :, 1] < 50)).astype(np.uint8)
            mask = cv2.resize(mask, (64, 48))
        else:
            bgr_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            mask = self.yolop_processor.process_image(bgr_image, (64, 48))

        mask_normalized = mask.astype(np.float32)
        tensor = torch.from_numpy(mask_normalized).unsqueeze(0).unsqueeze(0)
        return tensor.to(self.device), mask

    def timer_callback(self) -> None:
        if self.zed is None or not self.zed.grab():
            return
        cv_image = self.zed.get_image()
        if cv_image is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        input_tensor, mask = self.preprocess_image(cv_image)

        resized_input = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR), (64, 48))
        resized_input[mask == 1] = [0, 0, 255]
        debug_msg = self.bridge.cv2_to_imgmsg(resized_input, encoding='bgr8')
        debug_msg.header = header
        self.pub_debug_image.publish(debug_msg)

        with torch.no_grad():
            output = self.model(input_tensor)

        output_normalized = output.cpu().numpy().flatten()
        output_denormalized = denormalize_waypoints(output_normalized)
        output_denormalized_tensor = torch.from_numpy(output_denormalized).unsqueeze(0)

        path_raw_msg = self.create_path_from_output(output_denormalized_tensor, header)
        self.pub_raw.publish(path_raw_msg)

        path_smooth_msg = self.apply_bspline_smoothing(output_denormalized_tensor, header)
        self.pub.publish(path_smooth_msg)

    def pcl_publisher_callback(self) -> None:
        if self.zed is None or not self.zed.grab():
            return
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
