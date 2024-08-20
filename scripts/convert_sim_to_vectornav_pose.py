import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

import pymap3d as pm
import copy

class Converter(Node):
    def __init__(self):
        super().__init__("sim_to_vectornav_pose_converter_node")

        self.create_subscription(NavSatFix, "/gps", self.callback_gps, 10)
        self.create_subscription(Imu, "/imu", self.callback_imu, 10)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/vectornav/pose', 10)

        self.pub_msg = PoseWithCovarianceStamped()

        self.get_logger().info("シミュレータのgpsとimuからvectornav/poseを再現します．")



    def callback_gps(self, msg):
        x,y,z = pm.geodetic2ecef(msg.latitude, msg.longitude, msg.altitude)
        self.pub_msg.pose.pose.position.x = x
        self.pub_msg.pose.pose.position.y = y
        self.pub_msg.pose.pose.position.z = z


    def callback_imu(self, msg):
        self.pub_msg.header = msg.header
        self.pub_msg.pose.pose.orientation = msg.orientation

        # ENUにする
        # self.pub_msg.pose.pose.orientation.x = -msg.orientation.x
        # self.pub_msg.pose.pose.orientation.w = -msg.orientation.w

        self.publisher.publish(self.pub_msg)




def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
