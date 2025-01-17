import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

class Converter(Node):
    def __init__(self):
        super().__init__("gps_csv_node")
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, "/vectornav/pose", self.callback, 10)
        self.subscription
        self.get_logger().info("vectornav/poseをcsvファイルにします。")

        self.csv_file = open('default_name.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.counter = 0  # カウンタを初期化

    def callback(self, msg):
        self.counter += 1  # コールバックが呼ばれるたびにカウンタをインクリメント
        if self.counter % 1000 == 0:  # 10回に1度だけ書き込み
            latitude = msg.pose.pose.position.x
            longitude = msg.pose.pose.position.y
            ellipsoid = msg.pose.pose.position.z
            print(latitude, longitude, ellipsoid)
            self.csv_writer.writerow([latitude, longitude, ellipsoid])
            # self.get_logger().info(f'Latitude: {latitude}, Longitude: {longitude}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

