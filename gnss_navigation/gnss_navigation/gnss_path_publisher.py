import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pyproj import Proj, transform
import pandas as pd
from scipy.interpolate import splprep, splev
import numpy as np

class gnss_path_Publisher(Node):
    def __init__(self):
        super().__init__('gnss_path_Publisher')

        # WGS84座標系（GPSの座標系）
        self.wgs84 = Proj(init='epsg:4326')

        # UTM座標系への変換
        self.utm = Proj(init='epsg:32654')  # UTMゾーンに応じてEPSGコードを変更

        # parameterの設定 
        self.declare_parameter('file_path', '/home/ros2_ws/src/AIFormula_private/gnss_navigation/config/course_data/gazebo_shihou_course.csv')
        file_path = self.get_parameter('file_path').get_parameter_value().string_value
       
        df = pd.read_csv(file_path, header=None)     
        
        self.path = Path()
        self.smooth_path_ = Path()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'map'
        
        for index, data in df.iterrows():
            # 緯度経度をロボット座標系に変換
            x, y = self.convert_gps_to_utm(data[1], data[0])

            if index == 0:
                base_x = x
                base_y = y
            
            # PoseStampedを作成し，経路に追加
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x - base_x
            pose.pose.position.y = y - base_y
            self.path.poses.append(pose)

        self.get_logger().info("data loaded")
        self.get_logger().info("publish start")
            
        # 経路の平滑化処理
        self.smooth_path()

        self.publisher_ = self.create_publisher(Path, 'gnss_path', 10)
        self.timer = self.create_timer(1, self.publish_path)
        
    def convert_gps_to_utm(self, lat, lon):
        x, y = transform(self.wgs84, self.utm, lat, lon)
        return x, y
        
    def smooth_path(self):
        x = [pose.pose.position.x for pose in self.path.poses]
        y = [pose.pose.position.y for pose in self.path.poses]

        # スプライン補間
        tck, u = splprep([x, y], s=0, per=0)
        u_new = np.linspace(u.min(), u.max(), 500)
        x_new, y_new = splev(u_new, tck, der=0)

        # 補間した座標から新しいパスを生成
        self.smooth_path_.header = self.path.header
        for x, y in zip(x_new, y_new):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            self.smooth_path_.poses.append(pose)
        
    def publish_path(self):
        self.publisher_.publish(self.smooth_path_)
        
def main(args=None):
    try:
        rclpy.init(args=args)
        pub = gnss_path_Publisher()
        rclpy.spin(pub)
    except KeyboardInterrupt:
        print("ctrl-C")
    finally:
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()