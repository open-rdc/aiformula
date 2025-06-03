#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from socketcan_interface_msg.msg import SocketcanIF

class CandataToCasterYaw(Node):
    def __init__(self):
        super().__init__('candata_to_caster_yaw')
        self.subscription = self.create_subscription(
            SocketcanIF,
            '/can_rx_11',
            self.listener_callback,
            10)
        self.get_logger().info("Listening to /can_rx_11 for caster yaw...")

    def listener_callback(self, msg: SocketcanIF):
        if msg.canid != 0x11:
            return  # 他のCAN IDは無視

        if msg.candlc < 1:
            self.get_logger().warn("CANデータ長が不正です")
            return

        raw_value = msg.candata[0]
        yaw_deg = (raw_value - 100) * 2.0
        self.get_logger().info(f"CAN ID: 0x11 - Raw: {raw_value} → Yaw: {yaw_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = CandataToCasterYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

