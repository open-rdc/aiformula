import rclpy
from rclpy.node import Node
from nav?msgs.msg import Odometry
import csv
import math

class VoCsvConvereter(Node):
    def __init__(self):
        super().__init__("vo_csv_node")
        self.subscription = self.create_subscription(
            Odometry,
           "/vo/odom",
           self.callback,
           10
        )
        self.subscription

       self.get_logger().info("/vo/odomをCSVファイルにします。")
       self.csv_file = open('shihou_vo.csv', mode ='w', newline" ")
       self.csv_writer.writerow(["x", "y", "z", "yaw"])
       self.counter = 0

    def callback(self,msg):
        self.counter += 1
        if self.counter % 10 != 0:
           return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        #yawに変換
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        #yaw
        siny_cosp = 2.0 * (qw * qz + qx *qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f"x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}")
        self.csv_writer.writerow([x, y,z, yaw])
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
