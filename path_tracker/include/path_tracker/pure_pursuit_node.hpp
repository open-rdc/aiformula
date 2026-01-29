#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include "path_tracker/visibility_control.h"

namespace path_tracker{

class PurePursuit : public rclcpp::Node {
public:
    PATH_TRACKER_PUBLIC
    explicit PurePursuit(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    PATH_TRACKER_PUBLIC
    explicit PurePursuit(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _subscription_path;
    void _subscriber_callback_path(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_autonomous;
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_vel;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    bool autonomous_flag_ = false;

    const double linear_max_vel;
    const double lookahead_distance;
    const double wheelbase_;
    const double caster_max_angle_rad_;
};

}  // namespace path_tracker
