#ifndef UTIL_HPP
#define UTIL_HPP

// ROS
#include <tf2/impl/utils.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

namespace aiformula {

inline double toTimeStampDouble(const std_msgs::msg::Header& header) {
    return header.stamp.sec + static_cast<double>(header.stamp.nanosec) / 1e9;
}

inline double getYaw(const geometry_msgs::msg::Quaternion& quat_msg) {
    return tf2::impl::getYaw(tf2::impl::toQuaternion(quat_msg));
}

}  // namespace aiformula

#endif  // UTIL_HPP
