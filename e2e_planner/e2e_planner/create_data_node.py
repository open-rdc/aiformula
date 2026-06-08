import copy
import csv
import os
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import pymap3d as pm
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion

from e2e_planner.util.sensor_utils import SourceConfig, create_sensor_source

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10


def package_data_dir() -> str:
    # 実行場所に依らず e2e_planner パッケージ直下の data ディレクトリを指す。
    # realpath で symlink-install 経由の `ros2 run` もソースツリーへ解決する。
    module_dir = os.path.dirname(os.path.realpath(__file__))  # e2e_planner/e2e_planner
    package_root = os.path.dirname(module_dir)                # e2e_planner
    return os.path.join(package_root, 'data')


class Sample:
    def __init__(self, image: np.ndarray, timestamp: float, reference_pose: PoseWithCovarianceStamped):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        self.reference_pose: PoseWithCovarianceStamped = reference_pose
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]


class DataCollectionNode(Node):
    def __init__(self) -> None:
        super().__init__('create_data_node')

        # pyzed があれば ZED SDK 直叩き、無ければ ROS topic 購読へ自動フォールバック
        self.init_image_source()
        self.init_subscription()
        self.init_timer()

        self.latest_pose: Optional[PoseWithCovarianceStamped] = None
        self.samples: List[Sample] = []
        self.pose_history: Deque[Tuple[float, PoseWithCovarianceStamped]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]]]] = []
        self.last_sample_time: Optional[float] = None
        self.is_paused: bool = True
        self.prev_button_state: int = 0

        self.get_logger().info('⚪Create data started')

    def init_image_source(self) -> None:
        config = SourceConfig(depth_mode='PERFORMANCE')
        self.image_source = create_sensor_source(self, config)

    def init_subscription(self) -> None:
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/vectornav/pose', self.pose_callback, qos_profile_sensor_data
        )
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def init_timer(self) -> None:
        self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg

    def joy_callback(self, msg: Joy) -> None:
        if len(msg.buttons) <= 2:
            return
        current_button_state = msg.buttons[2]
        if current_button_state == 1 and self.prev_button_state == 0:
            self.is_paused = not self.is_paused
            if self.is_paused:
                self.get_logger().info('⏸️ Data collection paused')
            else:
                self.pose_history.clear()
                self.samples.clear()
                self.last_sample_time = None
                self.get_logger().info('▶️ Data collection resumed')
        self.prev_button_state = current_button_state

    def timer_callback(self) -> None:
        if self.is_paused:
            return

        current_time = time.time()
        if self.latest_pose is not None:
            self.pose_history.append((current_time, copy.deepcopy(self.latest_pose)))

        self.try_capture_sample(current_time)
        self.update_samples()
        self.cleanup_pose_history()

    def try_capture_sample(self, current_time: float) -> None:
        if not self.image_source.grab() or self.latest_pose is None:
            return
        if self.last_sample_time is not None and current_time - self.last_sample_time < SAMPLE_INTERVAL:
            return

        image = self.image_source.get_image()
        if image is None:
            return
        height, width = image.shape[:2]
        image = cv2.resize(image, (width // 2, height // 2))

        sample = Sample(image, current_time, copy.deepcopy(self.latest_pose))
        self.samples.append(sample)
        self.last_sample_time = current_time

    def update_samples(self) -> None:
        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints))
            self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            pose = self.find_closest_pose(target_time)
            if pose is not None:
                x, y = self.transform_to_robot_frame(sample.reference_pose, pose)
                sample.waypoints.append((x, y))
            else:
                break

    def find_closest_pose(self, target_time: float) -> Optional[PoseWithCovarianceStamped]:
        for t, pose in self.pose_history:
            if t >= target_time:
                return pose
        return None

    def cleanup_pose_history(self) -> None:
        if not self.samples or not self.pose_history:
            return
        incomplete_samples = [s for s in self.samples if len(s.waypoints) < NUM_WAYPOINTS]
        if not incomplete_samples:
            return
        min_target_time = min(sample.target_times[len(sample.waypoints)] for sample in incomplete_samples)
        while self.pose_history and self.pose_history[0][0] < min_target_time:
            self.pose_history.popleft()

    def transform_to_robot_frame(self, reference_pose: PoseWithCovarianceStamped,
                                 current_pose: PoseWithCovarianceStamped) -> Tuple[float, float]:
        x0_ecef = reference_pose.pose.pose.position.x
        y0_ecef = reference_pose.pose.pose.position.y
        z0_ecef = reference_pose.pose.pose.position.z
        lat0, lon0, alt0 = pm.ecef2geodetic(x0_ecef, y0_ecef, z0_ecef)

        q0 = reference_pose.pose.pose.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])

        xi_ecef = current_pose.pose.pose.position.x
        yi_ecef = current_pose.pose.pose.position.y
        zi_ecef = current_pose.pose.pose.position.z
        e, n, u = pm.ecef2enu(xi_ecef, yi_ecef, zi_ecef, lat0, lon0, alt0)

        x_robot = -e * np.sin(yaw0) + n * np.cos(yaw0)
        y_robot = -e * np.cos(yaw0) - n * np.sin(yaw0)

        return x_robot, y_robot

    def save_data(self) -> None:
        if len(self.collected_data) == 0:
            self.get_logger().info('🔴No data to save')
            return

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = os.path.join(package_data_dir(), f'{timestamp}_dataset')
        images_dir = os.path.join(dataset_dir, 'images')
        path_dir = os.path.join(dataset_dir, 'path')

        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(path_dir, exist_ok=True)

        for idx, (image, waypoints) in enumerate(self.collected_data, start=1):
            image_path = os.path.join(images_dir, f'{idx:05d}.png')
            waypoints_path = os.path.join(path_dir, f'{idx:05d}.csv')

            cv2.imwrite(image_path, image)

            with open(waypoints_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['x', 'y'])
                for x, y in waypoints:
                    csv_writer.writerow([x, y])

        self.get_logger().info(f'🔵Saved {len(self.collected_data)} samples to {dataset_dir}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
