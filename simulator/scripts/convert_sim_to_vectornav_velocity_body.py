#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class Converter(Node):
    def __init__(self):
        super().__init__("sim_to_vectornav_velocity_body_converter_node")
        self.declare_parameter("frame_id", "vectornav")
        self.body_frame_id = self.get_parameter("frame_id").value

        self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, "/vectornav/velocity_body", 10)

        self.get_logger().info("/odom から /vectornav/velocity_body を再現して出版します．")

    def callback_odom(self, msg: Odometry):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.body_frame_id

        out.twist.twist = msg.twist.twist
        out.twist.covariance = msg.twist.covariance

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
