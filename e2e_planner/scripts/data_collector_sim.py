#!/usr/bin/env python3
"""
シミュレーション用データ収集ノード。
ジョイコンで走行しながら左折/右折/道なりのラベルを付けてRGB画像を保存する。

ボタンマッピング:
  buttons[1] = 右折 (right)
  buttons[2] = 左折 (left)
  ボタン未押下 = 道なり (straight)

録画操作:
  buttons[0] = 録画開始/停止トグル

保存形式:
  ~/ros2_ws/dataset/session_YYYYMMDD_HHMMSS/
    images/000001.jpg  ...  (RGB 480x300)
    data.csv           (frame_id, image, timestamp, x, y, yaw, label)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Optional
def euler_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

LABEL_STRAIGHT = 0
LABEL_RIGHT    = 1
LABEL_LEFT     = 2
LABEL_NAMES    = {0: 'straight', 1: 'right', 2: 'left'}

BTN_RECORD   = 0
BTN_RIGHT    = 1
BTN_LEFT     = 2


class DataCollectorSim(Node):
    def __init__(self) -> None:
        super().__init__('data_collector_sim')

        self.declare_parameter('save_dir', str(Path.home() / 'ros2_ws' / 'dataset'))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('save_interval_sec', 0.2)  # 5Hz

        save_base = Path(self.get_parameter('save_dir').value)
        session_name = 'session_' + datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = save_base / session_name
        self.image_dir = self.session_dir / 'images'

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_odom: Optional[tuple] = None  # (x, y, yaw)
        self.current_label: int = LABEL_STRAIGHT
        self.recording: bool = False
        self.frame_id: int = 0
        self.last_save_time: float = 0.0
        self.prev_btn_record: int = 0

        self.csv_file = None
        self.csv_writer = None

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

        self.timer = self.create_timer(self.save_interval, self.save_callback)

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
        self.image_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.session_dir / 'data.csv'
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['frame_id', 'image', 'timestamp', 'x', 'y', 'yaw', 'label', 'label_name'])
        self.recording = True
        self.frame_id = 0
        self.get_logger().info(f'録画開始: {self.session_dir}')

    def _stop_recording(self) -> None:
        self.recording = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.get_logger().info(
            f'録画停止。保存フレーム数: {self.frame_id}  保存先: {self.session_dir}')

    def save_callback(self) -> None:
        if not self.recording:
            return
        if self.latest_image is None or self.latest_odom is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_save_time < self.save_interval * 0.9:
            return
        self.last_save_time = now

        self.frame_id += 1
        filename = f'{self.frame_id:06d}.jpg'
        image_path = self.image_dir / filename
        cv2.imwrite(str(image_path), self.latest_image)

        x, y, yaw = self.latest_odom
        self.csv_writer.writerow([
            self.frame_id,
            f'images/{filename}',
            now,
            round(x, 4), round(y, 4), round(yaw, 4),
            self.current_label,
            LABEL_NAMES[self.current_label],
        ])

        # 視覚フィードバック
        if self.frame_id % 50 == 0:
            self.get_logger().info(
                f'フレーム {self.frame_id}  ラベル={LABEL_NAMES[self.current_label]}'
                f'  pos=({x:.2f},{y:.2f})')

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
