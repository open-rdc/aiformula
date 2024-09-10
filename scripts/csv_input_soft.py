import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv

class Converter(Node):
    def __init__(self):
        super().__init__("gps_csv_node")
        self.subscription = self.create_subscription(NavSatFix, "/vectornav/gnss", self.callback, 10)
        self.subscription
        self.get_logger().info("vectornav/gnssをcsvファイルにします。")

        self.csv_file = open('shihou_gnssnav.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Latitude', 'Longitude'])

        self.counter = 0  # カウンタを初期化

    def callback(self, msg):
        self.counter += 1  # コールバックが呼ばれるたびにカウンタをインクリメント
        if self.counter % 10 == 0:  # 10回に1度だけ書き込み
            latitude = msg.latitude
            longitude = msg.longitude
            print(latitude, longitude)
            self.csv_writer.writerow([latitude, longitude])
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

