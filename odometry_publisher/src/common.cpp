#include "common.hpp"

namespace aiformula {
namespace odometry_publisher {

void broadcastTf(std::unique_ptr<tf2_ros::TransformBroadcaster>& odometry_br, const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header = odom.header;
    ts.child_frame_id = odom.child_frame_id;
    ts.transform.translation = toVector3Msg(odom.pose.pose.position);
    ts.transform.rotation = odom.pose.pose.orientation;
    odometry_br->sendTransform(ts);
}

}  // namespace odometry_publisher

geometry_msgs::msg::Point toPointMsg(const double& x, const double& y, const double& z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Quaternion toQuaternionMsg(const double& roll, const double& pitch, const double& yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Vector3 toVector3Msg(const double& x, const double& y, const double& z) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}

geometry_msgs::msg::Vector3 toVector3Msg(const geometry_msgs::msg::Point& point) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = point.x;
    vec.y = point.y;
    vec.z = point.z;
    return vec;
}

}  // namespace aiformula
