#ifndef COMMON_HPP
#define COMMON_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

// ROS msg
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "to_geometry_msgs.hpp"  // common_cpp librtary

namespace aiformula {
namespace odometry_publisher {

template <typename T>
struct Wheel {
    Wheel(const T& left, const T& right) : left(left), right(right) {}
    Wheel<double> operator*(const double& rhs) const { return {left * rhs, right * rhs}; }
    T left, right;
};
const double MINUTE_TO_SECOND = 0.016667;  // = 1/60
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;

const int RPM_ID = 1809;

void broadcastTf(std::unique_ptr<tf2_ros::TransformBroadcaster>& odometry_br, const nav_msgs::msg::Odometry& odom);

}  // namespace odometry_publisher
}  // namespace aiformula

#endif  // COMMON_HPP
