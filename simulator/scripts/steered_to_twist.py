#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from steered_drive_msg.msg import SteeredDrive

from steering_actuator_model import SteeringActuatorModel, SteeringActuatorParams


class SteeredToTwist(Node):
    def __init__(self) -> None:
        super().__init__('steered_to_twist')

        self.declare_parameter('wheel_base', 0.8)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_twist')
        self.declare_parameter('caster_topic', '/cmd_caster')
        self.declare_parameter('caster_data_topic', '/caster_data')
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('steer_dead_time', 0.04)
        self.declare_parameter('steer_tau', 0.12)
        self.declare_parameter('steer_gain', 0.9)
        self.declare_parameter('steer_rate_max_deg', 80.0)
        self.declare_parameter('a_lat_max', 4.0)
        self.declare_parameter('stop_velocity', 0.1)

        self._wheel_base = float(self.get_parameter('wheel_base').get_parameter_value().double_value)
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        caster_topic = self.get_parameter('caster_topic').get_parameter_value().string_value
        caster_data_topic = self.get_parameter('caster_data_topic').value

        if self._wheel_base <= 0.0:
            self.get_logger().error('wheel_base must be positive; forcing 1.0')
            self._wheel_base = 1.0

        self.get_logger().info(f'Using wheel_base: {self._wheel_base:.4f} m')

        self._model = SteeringActuatorModel(SteeringActuatorParams(
            dead_time=self.get_parameter('steer_dead_time').value,
            tau=self.get_parameter('steer_tau').value,
            gain=self.get_parameter('steer_gain').value,
            rate_max=math.radians(self.get_parameter('steer_rate_max_deg').value),
            a_lat_max=self.get_parameter('a_lat_max').value,
            wheel_base=self._wheel_base,
            stop_velocity=self.get_parameter('stop_velocity').value,
        ))
        self._cmd_velocity = 0.0
        self._cmd_steering = 0.0

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self._caster_pub = self.create_publisher(Float64MultiArray, caster_topic, 10)
        self._caster_data_pub = self.create_publisher(Float64MultiArray, caster_data_topic, 10)
        self._sub = self.create_subscription(SteeredDrive, input_topic, self.cmd_callback, 10)
        self._timer = self.create_timer(1.0 / self.get_parameter('update_rate').value, self._on_timer)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_callback(self, msg: SteeredDrive) -> None:
        self._cmd_velocity = float(msg.velocity)
        self._cmd_steering = float(msg.steering_angle)
        self._model.set_command(self._now(), self._cmd_steering)

    def _on_timer(self) -> None:
        delta_eff = self._model.update(self._now(), self._cmd_velocity)

        twist = Twist()
        twist.linear.x = self._cmd_velocity
        twist.angular.z = self._model.yaw_rate(self._cmd_velocity, delta_eff)
        self._pub.publish(twist)

        caster_cmd = Float64MultiArray()
        caster_cmd.data = [delta_eff * -1.0]
        self._caster_pub.publish(caster_cmd)

        caster_data = Float64MultiArray()
        caster_data.data = [self._cmd_steering, delta_eff, 0.0]
        self._caster_data_pub.publish(caster_data)


def main() -> None:
    rclpy.init()
    node = SteeredToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
