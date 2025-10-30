#!/usr/bin/env python3
"""
ROS2 レイテンシ計測スクリプト
パブリッシャーとサブスクライバーの両方を1つのノードで実装し、
送信時刻と受信時刻の差分からレイテンシを計測します
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import time


class LatencyMeasurement(Node):
    def __init__(self):
        super().__init__('latency_measurement')
        
        # パブリッシャーとサブスクライバーの作成
        self.publisher = self.create_publisher(Header, 'test_pub', 10)
        self.subscription = self.create_subscription(
            Header,
            'test_pub',  # 同じトピックをサブスクライブ
            self.listener_callback,
            10
        )
        
        # タイマーで定期的にメッセージを送信
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        # レイテンシ統計用
        self.latencies = []
        
        self.get_logger().info('レイテンシ計測ノードを起動しました')
        self.get_logger().info('Ctrl+C で終了すると統計情報を表示します')

    def timer_callback(self):
        msg = Header()
        msg.frame_id = str(self.count)
        
        # 現在時刻をタイムスタンプとして設定
        current_time = self.get_clock().now()
        msg.stamp = current_time.to_msg()
        
        self.publisher.publish(msg)
        self.get_logger().info(f'送信 #{self.count}')
        self.count += 1

    def listener_callback(self, msg):
        # 受信時刻を取得
        receive_time = self.get_clock().now()
        
        # 送信時刻を取得
        send_time = rclpy.time.Time.from_msg(msg.stamp)
        
        # レイテンシを計算（ナノ秒単位）
        latency_ns = (receive_time - send_time).nanoseconds
        latency_ms = latency_ns / 1_000_000.0  # ミリ秒に変換
        
        self.latencies.append(latency_ms)
        
        self.get_logger().info(
            f'受信 #{msg.frame_id}: レイテンシ = {latency_ms:.3f} ms'
        )

    def print_statistics(self):
        if not self.latencies:
            self.get_logger().info('データがありません')
            return
        
        avg_latency = sum(self.latencies) / len(self.latencies)
        min_latency = min(self.latencies)
        max_latency = max(self.latencies)
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('レイテンシ統計')
        self.get_logger().info('='*50)
        self.get_logger().info(f'計測回数: {len(self.latencies)}')
        self.get_logger().info(f'平均: {avg_latency:.3f} ms')
        self.get_logger().info(f'最小: {min_latency:.3f} ms')
        self.get_logger().info(f'最大: {max_latency:.3f} ms')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = LatencyMeasurement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_statistics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
