#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import csv
from pathlib import Path
from collections import deque
from typing import Optional, List, Tuple, Deque
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10

class Sample:
    def __init__(self, image: np.ndarray, timestamp: float, reference_odom: Odometry):
        self.image: np.ndarray = image
        self.timestamp: float = timestamp
        self.reference_odom: Odometry = reference_odom
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times: List[float] = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]

class DataCollectionNode(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_node')

        self.bridge: CvBridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Odometry] = None

        self.samples: List[Sample] = []
        self.odom_history: Deque[Tuple[float, Odometry]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]]]] = []
        self.last_sample_time: Optional[float] = None

        self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('⚪Create data started')

    def image_callback(self, msg: Image) -> None:
        self.latest_image = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def timer_callback(self) -> None:
        current_time = time.time()

        if self.latest_odom is not None:
            self.odom_history.append((current_time, self.latest_odom))

        if self.latest_image is not None and self.latest_odom is not None:
            if self.last_sample_time is None or current_time - self.last_sample_time >= SAMPLE_INTERVAL:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgra8')
                sample = Sample(cv_image, current_time, self.latest_odom)
                self.samples.append(sample)
                self.last_sample_time = current_time

        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        remaining = []
        for sample in self.samples:
            if len(sample.waypoints) == NUM_WAYPOINTS:
                self.collected_data.append((sample.image, sample.waypoints))
                self.get_logger().info(f'🟡Collected data #{len(self.collected_data)}')
            else:
                remaining.append(sample)
        self.samples = remaining

        self.cleanup_odom_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            target_time = sample.target_times[i]
            odom = self.find_closest_odom(target_time)
            if odom is not None:
                x, y = self.transform_to_robot_frame(sample.reference_odom, odom)
                sample.waypoints.append((x, y))
            else:
                break

    def find_closest_odom(self, target_time: float) -> Optional[Odometry]:
        for t, odom in self.odom_history:
            if t >= target_time:
                return odom
        return None

    def cleanup_odom_history(self) -> None:
        if not self.samples or not self.odom_history:
            return
        incomplete_samples = [s for s in self.samples if len(s.waypoints) < NUM_WAYPOINTS]
        if not incomplete_samples:
            return
        min_target_time = min(sample.target_times[len(sample.waypoints)] for sample in incomplete_samples)
        while self.odom_history and self.odom_history[0][0] < min_target_time:
            self.odom_history.popleft()

    def transform_to_robot_frame(self, reference_odom: Odometry, odom: Odometry) -> Tuple[float, float]:
        x0 = reference_odom.pose.pose.position.x
        y0 = reference_odom.pose.pose.position.y
        q0 = reference_odom.pose.pose.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])

        xi = odom.pose.pose.position.x
        yi = odom.pose.pose.position.y

        dx = xi - x0
        dy = yi - y0

        x_robot = dx * np.cos(-yaw0) - dy * np.sin(-yaw0)
        y_robot = dx * np.sin(-yaw0) + dy * np.cos(-yaw0)

        return x_robot, y_robot

    def save_data(self) -> None:
        if len(self.collected_data) == 0:
            self.get_logger().info('🔴No data to save')
            return

        package_root = Path(__file__).parent.parent
        data_base_dir = package_root / 'data'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = data_base_dir / f'{timestamp}_dataset'
        images_dir = dataset_dir / 'images'
        path_dir = dataset_dir / 'path'

        images_dir.mkdir(parents=True, exist_ok=True)
        path_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            csv_path = path_dir / f'{idx:05d}.csv'

            cv2.imwrite(str(image_path), image)

            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y'])
                writer.writerows([[f'{x:.3f}', f'{y:.3f}'] for x, y in waypoints])

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
