import os
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.interpolate import splev, splprep
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import torch

from e2e_planner.util.sensor_utils import create_sensor_source, filtered_pointcloud
from e2e_planner.util.yolop_processor import YOLOPv2Processor


def denormalize_waypoints(normalized: np.ndarray) -> np.ndarray:
    denormalized = normalized.copy()
    denormalized[0::2] = (normalized[0::2] + 1.0) * 5.0  # x座標
    denormalized[1::2] = (normalized[1::2] + 1.0) * 3.0 - 3.0  # y座標
    return denormalized


class InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('inference_node')
        self.init_ros_parameter()
        self.init_torch_model()

        self.bridge = CvBridge()
        self.cv_image = None
        # pyzed があれば ZED SDK 直叩き、無ければ ROS topic 購読へ自動フォールバック
        self.image_source = create_sensor_source(self)

        self.init_publisher()
        self.init_timer()

    def init_ros_parameter(self) -> None:
        self.declare_parameter('model_name', 'model.pt')
        self.declare_parameter('interval_ms', 100)
        self.model_name = self.get_parameter('model_name').value
        self.interval_ms = int(self.get_parameter('interval_ms').value)

    def init_torch_model(self) -> None:
        package_share_directory = get_package_share_directory('e2e_planner')
        weight_path = os.path.join(package_share_directory, 'weights', self.model_name)
        yolop_weight_path = os.path.join(package_share_directory, 'weights', 'yolopv2.pt')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using torch device: {self.device}')

        self.model = torch.jit.load(weight_path, map_location=self.device)
        self.model.eval()

        self.yolop_processor = YOLOPv2Processor(yolop_weight_path, self.device)

    def init_publisher(self) -> None:
        self.publisher_path_raw = self.create_publisher(Path, 'e2e_planner/path_raw', qos_profile_system_default)
        self.publisher_path = self.create_publisher(Path, 'e2e_planner/path', qos_profile_system_default)
        self.publisher_pointcloud = self.create_publisher(
            PointCloud2, '/zed/zed_node/pointcloud_filtered', qos_profile_sensor_data
        )
        self.publisher_debug_image = self.create_publisher(Image, 'e2e_planner/debug_image', qos_profile_system_default)

    def init_timer(self) -> None:
        self.torch_cb_group = ReentrantCallbackGroup()
        self.zed_cb_group = ReentrantCallbackGroup()
        self.torch_timer = self.create_timer(
            self.interval_ms / 1000.0,
            self.torch_callback,
            callback_group=self.torch_cb_group,
        )
        self.zed_timer = self.create_timer(
            10.0 / 1000.0,
            self.zed_sensor_callback,
            callback_group=self.zed_cb_group,
        )

    def create_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        return header

    def preprocess_image(self, image: np.ndarray) -> Tuple[torch.Tensor, np.ndarray]:
        bgr_image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        mask = self.yolop_processor.process_image(bgr_image, (64, 48))

        mask_normalized = mask.astype(np.float32)
        tensor = torch.from_numpy(mask_normalized).unsqueeze(0).unsqueeze(0)
        return tensor.to(self.device), mask

    def torch_callback(self) -> None:
        if self.cv_image is None:
            return
        header = self.create_header()

        input_tensor, mask = self.preprocess_image(self.cv_image)
        self.publish_debug_image(self.cv_image, mask, header)

        with torch.no_grad():
            output = self.model(input_tensor)

        output = torch.from_numpy(denormalize_waypoints(output.cpu().numpy().flatten())).unsqueeze(0)

        self.publisher_path_raw.publish(self.create_path_from_output(output, header))
        self.publisher_path.publish(self.apply_bspline_smoothing(output, header))

    def zed_sensor_callback(self) -> None:
        if not self.image_source.grab():
            return

        self.cv_image = self.image_source.get_image()
        header = self.create_header()

        raw_cloud = self.image_source.get_pointcloud()
        pointcloud_msg = filtered_pointcloud(header, raw_cloud) if raw_cloud is not None else None
        if pointcloud_msg is None:
            pointcloud_msg = PointCloud2()
            pointcloud_msg.header = header
        self.publisher_pointcloud.publish(pointcloud_msg)

    def publish_debug_image(self, image: np.ndarray, mask: np.ndarray, header: Header) -> None:
        resized_input = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGRA2BGR), (64, 48))
        resized_input[mask == 1] = [0, 0, 255]
        debug_msg = self.bridge.cv2_to_imgmsg(resized_input, encoding='bgr8')
        debug_msg.header = header
        self.publisher_debug_image.publish(debug_msg)

    def apply_bspline_smoothing(self, output: torch.Tensor, header: Header) -> Path:
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
        path_msg.poses.extend(
            PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x_smooth[i]), y=float(y_smooth[i]))))
            for i in range(len(x_smooth))
        )

        return path_msg

    def create_path_from_output(self, output: torch.Tensor, header: Header) -> Path:
        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = 'base_link'
        waypoints = output.cpu().numpy().reshape(-1, 2)

        path_msg.poses.append(PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=0.0, y=0.0))))
        path_msg.poses.extend(
            PoseStamped(header=path_msg.header, pose=Pose(position=Point(x=float(x), y=float(y))))
            for x, y in waypoints
        )

        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InferenceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
