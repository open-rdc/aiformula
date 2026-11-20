#!/usr/bin/env python3
"""1 周ごとにパイロンを置き直す (学習データの収集用)。

PylonControl (/pylon/setting) は、起動時の車の位置と信号機のまわりを避けてパイロンを置く。
走行中に呼ぶと車のすぐ前に置かれうるので、車がスタート地点の近くに戻ってきたときだけ呼ぶ。

    ros2 run simulator pylon_reshuffle.py
    (パラメータ: start_radius_m スタート地点とみなす半径, leave_radius_m 1 周に出たとみなす距離)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty


class PylonReshuffle(Node):
    def __init__(self) -> None:
        super().__init__('pylon_reshuffle')
        self.start_radius_m = self.declare_parameter('start_radius_m', 6.0).value
        self.leave_radius_m = self.declare_parameter('leave_radius_m', 30.0).value
        self.client = self.create_client(Empty, '/pylon/setting')
        # /odom は Gazebo の車の位置 (ワールド座標)。スタート地点は最初に受け取った位置
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.start = None
        self.left_start = False
        self.laps = 0
        self.placed_first = False

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self.start is None:
            self.start = (p.x, p.y)
            self.get_logger().info(f'スタート地点 ({p.x:.1f}, {p.y:.1f})')
        if not self.placed_first:
            self.placed_first = self.call('最初の配置')
            return
        dist = math.hypot(p.x - self.start[0], p.y - self.start[1])
        if dist > self.leave_radius_m:
            self.left_start = True
        elif self.left_start and dist < self.start_radius_m:
            self.left_start = False
            self.laps += 1
            self.call(f'{self.laps} 周目が終わったので置き直す')

    def call(self, reason: str) -> bool:
        if not self.client.service_is_ready():
            self.get_logger().warn('/pylon/setting がまだ無い', throttle_duration_sec=5.0)
            return False
        self.client.call_async(Empty.Request())
        self.get_logger().info(f'パイロンを置いた ({reason})')
        return True


def main() -> None:
    rclpy.init()
    node = PylonReshuffle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
