import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
# from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Pose
import copy
import numpy as np

class Converter(Node):
    def __init__(self):
        super().__init__("vectornav_pose_to_euler_node")

        self.create_subscription(PoseWithCovarianceStamped, "/vectornav/pose", self.callback, 10)

        self.get_logger().info("vectornav poseをオイラー角にしてプリントします．")


    def callback(self, msg):
        roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        # self.get_logger().info(yaw)
        self.get_logger().info("yaw : %lf °" % (np.rad2deg(yaw)))

        return

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw



def main():
    rclpy.init()
    node = Converter()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
