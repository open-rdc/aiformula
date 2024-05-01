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
}  // namespace aiformula
