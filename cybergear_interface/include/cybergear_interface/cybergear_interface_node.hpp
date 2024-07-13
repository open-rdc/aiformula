#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "cybergear_interface/driver.hpp"
#include "cybergear_interface/visibility_control.h"

namespace cybergear_interface{

class CybergearInterface : public rclcpp::Node {
public:
    CYBERGEAR_INTERFACE_PUBLIC
    explicit CybergearInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CYBERGEAR_INTERFACE_PUBLIC
    explicit CybergearInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_pos;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_pos(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _publisher_callback();

    rclcpp::QoS _qos = rclcpp::QoS(10);

    Driver driver;

    // 定数
    const int interval_ms;

    // 変数
    double init_speed = 0.0;
};

}  // namespace cybergear_interface
