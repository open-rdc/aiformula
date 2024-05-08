#ifndef WHEEL_ODOMETRY_HPP
#define WHEEL_ODOMETRY_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <nav_msgs/msg/odometry.hpp>

// Original
#include "common.hpp"

#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace aiformula {

class WheelOdometry : public rclcpp::Node {
public:
    WheelOdometry();
    ~WheelOdometry() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void canFrameCallback(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    std::string odom_frame_id_;
    std::string robot_frame_id_;
    double tire_diameter_;
    double tire_tread_;

    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr can_frame_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_br_;
};

}  // namespace aiformula

#endif  // WHEEL_ODOMETRY_HPP
