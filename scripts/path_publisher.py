#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu


class PathGenerator(Node):
    def __init__(self) -> None:
        super().__init__("path_generator")
        self.publisher = self.create_publisher(Path, "/e2e_planner/path", 10)
        self.imu_sub = self.create_subscription(Imu, "/vectornav/imu", self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.base_points = [(float(x), 0.0) for x in range(0, 11)]
        self.initial_yaw = None
        self.current_yaw = None
        self.get_logger().info("開始位置を基準に， base_linkでカーブ経路の/frenet_planner/pathを出版します．")

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info("初期位置の設定が完了しました．")
        self.current_yaw = yaw

    def _build_path(self, yaw_delta: float) -> Path:
        path = Path()
        path.header.frame_id = "base_link"
        cos_yaw = math.cos(yaw_delta)
        sin_yaw = math.sin(yaw_delta)
        for x, y in self.base_points:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = (x * cos_yaw) - (y * sin_yaw)
            pose.pose.position.y = (x * sin_yaw) + (y * cos_yaw)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def timer_callback(self) -> None:
        if self.current_yaw is None or self.initial_yaw is None:
            return
        yaw_delta = self.initial_yaw - self.current_yaw
        self.path = self._build_path(yaw_delta)
        now = self.get_clock().now().to_msg()
        self.path.header.stamp = now
        for pose in self.path.poses:
            pose.header.stamp = now
        self.publisher.publish(self.path)


def main() -> None:
    rclpy.init()
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
