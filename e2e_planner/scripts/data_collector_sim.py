#!/usr/bin/env python3
"""
シミュレーション用データ収集ノード。
ジョイコンで走行しながら左折/右折/道なりのラベルを付けてRGB画像を保存する。

ボタンマッピング:
  buttons[7] = 右折 (right)
  buttons[6] = 左折 (left)
  ボタン未押下 = 道なり (straight)

録画操作:
  buttons[2] = 録画開始/停止トグル

保存形式:
  e2e_planner/data/YYYYMMDD_HHMMSS_dataset/
    images/00001.png
    path/00001.csv       (x, y)
    commands/00001.csv   (command)
    pointclouds/
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import copy
import math
import time
from pathlib import Path
from collections import deque
from typing import Deque, List, Optional, Tuple
def euler_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

SAMPLE_INTERVAL = 0.2
WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10

LABEL_ROADSIDE = 0
LABEL_STRAIGHT = 1
LABEL_LEFT     = 2
LABEL_RIGHT    = 3
LABEL_NAMES    = {0: 'roadside', 1: 'straight', 2: 'left', 3: 'right'}

BTN_RECORD   = 2
BTN_RIGHT    = 7
BTN_LEFT     = 6


class Sample:
    def __init__(self, image: np.ndarray, timestamp: float, reference_odom: Tuple[float, float, float], command: int):
        self.image = image
        self.timestamp = timestamp
        self.reference_odom = reference_odom
        self.command = command
        self.waypoints: List[Tuple[float, float]] = []
        self.target_times = [timestamp + WAYPOINT_INTERVAL * (i + 1) for i in range(NUM_WAYPOINTS)]


class DataCollectorSim(Node):
    def __init__(self) -> None:
        super().__init__('data_collector_sim')

        package_root = Path(__file__).parent.parent
        self.declare_parameter('save_dir', str(package_root / 'data'))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('save_interval_sec', SAMPLE_INTERVAL)  # 5Hz

        self.save_base_dir = Path(self.get_parameter('save_dir').value)

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_odom: Optional[Tuple[float, float, float]] = None
        self.current_label: int = LABEL_STRAIGHT
        self.recording: bool = False
        self.last_sample_time: Optional[float] = None
        self.prev_btn_record: int = 0
        self.samples: List[Sample] = []
        self.odom_history: Deque[Tuple[float, Tuple[float, float, float]]] = deque()
        self.collected_data: List[Tuple[np.ndarray, List[Tuple[float, float]], int]] = []

        image_topic = self.get_parameter('image_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        joy_topic = self.get_parameter('joy_topic').value
        self.save_interval = self.get_parameter('save_interval_sec').value

        self.sub_image = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_system_default)
        self.sub_joy = self.create_subscription(
            Joy, joy_topic, self.joy_callback, qos_profile_system_default)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Data collector ready.')
        self.get_logger().info(f'  buttons[{BTN_RECORD}]: 録画開始/停止')
        self.get_logger().info(f'  buttons[{BTN_RIGHT}]: 右折ラベル')
        self.get_logger().info(f'  buttons[{BTN_LEFT}]: 左折ラベル')
        self.get_logger().info(f'  未押下: 道なりラベル')

    def image_callback(self, msg: Image) -> None:
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_odom = (x, y, yaw)

    def joy_callback(self, msg: Joy) -> None:
        if len(msg.buttons) <= max(BTN_RECORD, BTN_RIGHT, BTN_LEFT):
            return

        # 録画トグル（ボタン立ち上がりエッジ）
        btn_record_now = msg.buttons[BTN_RECORD]
        if btn_record_now == 1 and self.prev_btn_record == 0:
            self._toggle_recording()
        self.prev_btn_record = btn_record_now

        # ラベル更新（押している間そのラベル）
        if msg.buttons[BTN_RIGHT] == 1:
            self.current_label = LABEL_RIGHT
        elif msg.buttons[BTN_LEFT] == 1:
            self.current_label = LABEL_LEFT
        else:
            self.current_label = LABEL_STRAIGHT

    def _toggle_recording(self) -> None:
        if not self.recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self) -> None:
        self.recording = True
        self.last_sample_time = None
        self.samples.clear()
        self.odom_history.clear()
        self.get_logger().info('録画開始')

    def _stop_recording(self) -> None:
        self.recording = False
        self.get_logger().info(
            f'録画停止。収集済みサンプル数: {len(self.collected_data)}')

    def timer_callback(self) -> None:
        if not self.recording:
            return
        if self.latest_image is None or self.latest_odom is None:
            return

        current_time = time.time()
        self.odom_history.append((current_time, copy.deepcopy(self.latest_odom)))

        if self.last_sample_time is None or current_time - self.last_sample_time >= self.save_interval:
            self.samples.append(
                Sample(copy.deepcopy(self.latest_image), current_time, copy.deepcopy(self.latest_odom), self.current_label)
            )
            self.last_sample_time = current_time

        for sample in self.samples:
            self.collect_waypoints_for_sample(sample)

        completed_samples = [sample for sample in self.samples if len(sample.waypoints) == NUM_WAYPOINTS]
        for sample in completed_samples:
            self.collected_data.append((sample.image, sample.waypoints, sample.command))
            if len(self.collected_data) % 50 == 0:
                self.get_logger().info(
                    f'サンプル {len(self.collected_data)}  ラベル={LABEL_NAMES[sample.command]}')

        self.samples = [sample for sample in self.samples if len(sample.waypoints) < NUM_WAYPOINTS]
        self.cleanup_odom_history()

    def collect_waypoints_for_sample(self, sample: Sample) -> None:
        for i in range(len(sample.waypoints), NUM_WAYPOINTS):
            odom = self.find_closest_odom(sample.target_times[i])
            if odom is None:
                break
            sample.waypoints.append(self.transform_to_robot_frame(sample.reference_odom, odom))

    def find_closest_odom(self, target_time: float) -> Optional[Tuple[float, float, float]]:
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

    def transform_to_robot_frame(
        self,
        reference_odom: Tuple[float, float, float],
        current_odom: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        x0, y0, yaw0 = reference_odom
        x, y, _ = current_odom
        dx = x - x0
        dy = y - y0
        x_robot = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        y_robot = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        return x_robot, y_robot

    def save_data(self) -> None:
        if len(self.collected_data) == 0:
            self.get_logger().info('保存するデータがありません')
            return

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        dataset_dir = self.save_base_dir / f'{timestamp}_dataset'
        images_dir = dataset_dir / 'images'
        path_dir = dataset_dir / 'path'
        commands_dir = dataset_dir / 'commands'
        pointclouds_dir = dataset_dir / 'pointclouds'

        images_dir.mkdir(parents=True, exist_ok=True)
        path_dir.mkdir(parents=True, exist_ok=True)
        commands_dir.mkdir(parents=True, exist_ok=True)
        pointclouds_dir.mkdir(parents=True, exist_ok=True)

        for idx, (image, waypoints, command) in enumerate(self.collected_data, start=1):
            image_path = images_dir / f'{idx:05d}.png'
            waypoints_path = path_dir / f'{idx:05d}.csv'
            command_path = commands_dir / f'{idx:05d}.csv'

            cv2.imwrite(str(image_path), image)

            with open(str(waypoints_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['x', 'y'])
                for x, y in waypoints:
                    csv_writer.writerow([x, y])

            with open(str(command_path), 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([command])

        self.get_logger().info(f'保存完了: {len(self.collected_data)} samples -> {dataset_dir}')

    def destroy_node(self) -> None:
        if self.recording:
            self._stop_recording()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
