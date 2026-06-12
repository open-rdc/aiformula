#!/usr/bin/env python3
import math

import pymap3d as pm
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, NavSatFix


class SimVectornavConverter(Node):
    """
    シミュレータのセンサデータを実機 VectorNav 相当に変換して再配信する。
    - /imu_raw       → yaw オフセット適用 → /vectornav/imu
    - /vectornav/gnss + /vectornav/imu(補正後) → ECEF 位置 + 補正姿勢 → /vectornav/pose
    """

    def __init__(self):
        super().__init__("sim_to_vectornav_converter_node")
        self.declare_parameter("yaw_offset_deg", -176.0)
        self.declare_parameter("imu_frame_id", "base_link")
        self.yaw_offset_deg = float(self.get_parameter("yaw_offset_deg").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)

        self._latest_corrected_imu: Imu | None = None

        self.create_subscription(Imu, "/imu_raw", self._imu_callback, 10)
        self.create_subscription(NavSatFix, "/vectornav/gnss", self._gnss_callback, 10)

        self._imu_pub  = self.create_publisher(Imu, "/vectornav/imu", 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/vectornav/pose", 10)

        self.get_logger().info(
            f"yaw オフセット {self.yaw_offset_deg} 度を適用し，"
            f"imu_frame_id={self.imu_frame_id} として "
            f"/vectornav/imu と /vectornav/pose を配信します"
        )

    # ------------------------------------------------------------------
    def _imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.imu_frame_id
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.orientation_covariance = msg.orientation_covariance

        q_in = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_out = self._apply_yaw_offset(q_in, self.yaw_offset_deg)
        out.orientation.x = q_out[0]
        out.orientation.y = q_out[1]
        out.orientation.z = q_out[2]
        out.orientation.w = q_out[3]

        self._latest_corrected_imu = out
        self._imu_pub.publish(out)

    def _gnss_callback(self, msg: NavSatFix):
        if self._latest_corrected_imu is None:
            return

        x, y, z = pm.geodetic2ecef(msg.latitude, msg.longitude, msg.altitude)

        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.header.frame_id = "earth"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = z
        pose.pose.pose.orientation = self._latest_corrected_imu.orientation

        self._pose_pub.publish(pose)

    # ------------------------------------------------------------------
    @staticmethod
    def _apply_yaw_offset(quaternion, yaw_degrees):
        rot = Rotation.from_quat(quaternion)
        euler = rot.as_euler('xyz', degrees=False)
        euler[2] += math.radians(yaw_degrees)
        return Rotation.from_euler('xyz', euler).as_quat()


def main():
    rclpy.init()
    node = SimVectornavConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
