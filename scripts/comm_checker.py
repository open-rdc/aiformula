import sys
import time
import socket
import struct
import rclpy
from rclpy.node import Node
from steered_drive_msg.msg import SteeredDrive

CAN_MAX_DLEN = 8
CAN_MTU = 16

CAN_NAME = 'can0'

class CanChecker():
    def __init__(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((CAN_NAME,))
        self.sock.settimeout(0.1)

    def can_receive(self):
        try:
            frame = self.sock.recv(16)
        except socket.timeout:
            return None
        return int.from_bytes(frame[:4], "little") & 0x1FFFFFFF

    def count(self, duration):
            counts = {}
            start = time.monotonic()
            while time.monotonic() - start < duration:
                can_id = self.can_receive()
                if can_id is None:
                    continue
                counts[can_id] = counts.get(can_id, 0) + 1
            return counts


class CommsChecker(Node):
    def __init__(self):
        super().__init__("comms_checker")
        self.subscription = self.create_subscription(SteeredDrive, "/cmd_vel",self._subscriber_callback_vel, 10)
        self.callback_count = 0
        self.get_logger().info("計測を開始します")


    def _subscriber_callback_vel(self, msg):
        self.callback_count += 1

def main():
    rclpy.init()
    node = CommsChecker()

    can = CanChecker()
    can_counts = can.count(2.0)  

    start_time = time.monotonic()
    while node.callback_count == 0 and time.monotonic() - start_time < 3.0:
        rclpy.spin_once(node, timeout_sec = 0.01)

    if node.callback_count == 0:
        print("topicが観測できませんでした")
        return

    start_time = time.monotonic()
    node.callback_count = 0
    while time.monotonic() - start_time < 3.0:
        rclpy.spin_once(node, timeout_sec = 0.01)
    elapsed = time.monotonic()

    node_hz =  node.callback_count / (elapsed - start_time)

    node.get_logger().info(f"/cmd_vel: {node_hz:.1f} Hz")

    for can_id, n in sorted(can_counts.items(), key=lambda x: -x[1]):
        print(f"0x{can_id:03X}  {n:5d} 件")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

