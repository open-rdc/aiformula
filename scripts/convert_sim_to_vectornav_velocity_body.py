#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped


class VelocityConverter(Node):
    def __init__(self):
        super().__init__("sim_to_vectornav_velocity_body_converter_node")

        # Subscriber: /odom topic
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Publisher: /vectornav/velocity_body topic
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/vectornav/velocity_body',
            10
        )

        self.get_logger().info("odomトピックからvectornav/velocity_bodyトピックへの変換を開始します。")


    def odom_callback(self, msg):
        """
        Odometryメッセージを受信し、TwistWithCovarianceStampedに変換してパブリッシュ

        Args:
            msg (Odometry): /odomトピックから受信したOdometryメッセージ
        """
        # TwistWithCovarianceStampedメッセージを作成
        velocity_msg = TwistWithCovarianceStamped()

        # ヘッダー情報をコピー
        velocity_msg.header = msg.header

        # Twistデータ（linear, angular velocity）をコピー
        velocity_msg.twist.twist.linear.x = msg.twist.twist.linear.x
        velocity_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        velocity_msg.twist.twist.linear.z = msg.twist.twist.linear.z

        velocity_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        velocity_msg.twist.twist.angular.y = msg.twist.twist.angular.y
        velocity_msg.twist.twist.angular.z = msg.twist.twist.angular.z

        # Covarianceは空（すべて0.0）で設定
        # ROS2のTwistWithCovarianceのcovarianceは6x6=36要素の配列
        velocity_msg.twist.covariance = [0.0] * 36

        # パブリッシュ
        self.publisher.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
