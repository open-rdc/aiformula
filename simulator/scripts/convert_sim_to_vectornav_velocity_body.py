#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class Converter(Node):
    def __init__(self):
        super().__init__("sim_to_vectornav_velocity_body_converter_node")
        self.declare_parameter("frame_id", "vectornav")
        self.declare_parameter("odom_child_frame_id", "ai_car1/chassis")
        self.body_frame_id = self.get_parameter("frame_id").value
        self.odom_child_frame_id = self.get_parameter("odom_child_frame_id").value

        self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, "/vectornav/velocity_body", 10)

        self.get_logger().info("/odom から /vectornav/velocity_body を再現して出版します．")

    def callback_odom(self, msg: Odometry):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.body_frame_id

        # 実機のvectornavドライバはVN body系(x前 / y右 / z下、yaw rateは時計回り正)で
        # velocity_bodyを出す。Gazeboの/odomはREP-103(x前 / y左 / z上)なので、
        # x軸まわりpi回転でVN body系に合わせる。
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = -msg.twist.twist.linear.y
        out.twist.twist.linear.z = -msg.twist.twist.linear.z
        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = -msg.twist.twist.angular.y
        out.twist.twist.angular.z = -msg.twist.twist.angular.z
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
