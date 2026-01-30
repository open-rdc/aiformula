#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class AckermannToTwist(Node):
    def __init__(self) -> None:
        super().__init__('ackermann_to_twist')

        self.declare_parameter('wheel_base', 0.8)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_twist')
        self.declare_parameter('caster_topic', '/cmd_caster')

        self._wheel_base = float(self.get_parameter('wheel_base').get_parameter_value().double_value)
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        caster_topic = self.get_parameter('caster_topic').get_parameter_value().string_value

        if self._wheel_base <= 0.0:
            self.get_logger().error('wheel_base must be positive; forcing 1.0')
            self._wheel_base = 1.0

        self.get_logger().info(f'Using wheel_base: {self._wheel_base:.4f} m')

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self._caster_pub = self.create_publisher(Float64MultiArray, caster_topic, 10)
        self._sub = self.create_subscription(AckermannDrive, input_topic, self.cmd_callback, 10)

    def cmd_callback(self, msg: AckermannDrive) -> None:
        linear_vel = float(msg.speed)
        steering_angle = float(msg.steering_angle)

        angular_vel = (linear_vel * math.tan(steering_angle)) / self._wheel_base

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self._pub.publish(twist)

        caster_cmd = Float64MultiArray()
        caster_cmd.data = [steering_angle*-1.0]
        self._caster_pub.publish(caster_cmd)


def main() -> None:
    rclpy.init()
    node = AckermannToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
