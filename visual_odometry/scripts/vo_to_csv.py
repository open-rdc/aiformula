import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import csv

class VoCsvConverter(Node):
    def __init__(self):
        super().__init__("vo_csv_node")
        self.subscription = self.create_subscription(
            Odometry,
           "/zed/zed_node/odom",
           self.callback,
           10
        )
        self.subscription

        self.get_logger().info("ZED VOをCSVファイルにします。")
        self.csv_file = open("shihou_vo.csv", mode ="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.counter = 0
        self.collecting = False
        self.prev_buttons = []

        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )
        self.joy_subscription

    def callback(self, msg):
        if not self.collecting:
            return

        self.counter += 1
        if self.counter % 100 != 0:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"x={x:.2f}, y={y:.2f}")
        self.csv_writer.writerow([x, y])

    def joy_callback(self, msg: Joy):
        # ボタン[1] の立ち上がりで収集開始
        buttons = msg.buttons
        if len(buttons) > 1:
            prev = 0
            if len(self.prev_buttons) > 1:
                prev = self.prev_buttons[1]

            if buttons[1] == 1 and prev == 0:
                self.collecting = False if self.collecting else True
                self.get_logger().info(f"ジョイスティックボタン[1]押下: データ収集{self.collecting}")

        self.prev_buttons = list(buttons)
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
        rclpy.init()
        node = VoCsvConverter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
