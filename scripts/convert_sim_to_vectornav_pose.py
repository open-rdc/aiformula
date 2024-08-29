import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

import pymap3d as pm
import math
import numpy as np
from scipy.spatial.transform import Rotation
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
        self.pub_msg.header = msg.header

        x,y,z = pm.geodetic2ecef(msg.latitude, msg.longitude, msg.altitude)
        self.pub_msg.pose.pose.position.x = x
        self.pub_msg.pose.pose.position.y = y
        self.pub_msg.pose.pose.position.z = z


    def callback_imu(self, msg):
        t = msg.orientation
        q = self.fix_rotation([t.x, t.y, t.z, t.w], -93.41)

        self.pub_msg.pose.pose.orientation.x = q[0]
        self.pub_msg.pose.pose.orientation.y = q[1]
        self.pub_msg.pose.pose.orientation.z = q[2]
        self.pub_msg.pose.pose.orientation.w = q[3]

        self.publisher.publish(self.pub_msg)


    def fix_rotation(self, quaternion, yaw_degrees):
        rotation = Rotation.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=False)
        euler[2] = euler[2] + np.deg2rad(yaw_degrees)

        # print(np.deg2rad(euler))
        fixed_rotation = Rotation.from_euler('xyz', euler)
        return fixed_rotation.as_quat()


def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
