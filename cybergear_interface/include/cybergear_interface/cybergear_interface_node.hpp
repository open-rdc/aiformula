#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscription_pos;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_reset;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_pos(const std_msgs::msg::Float64::SharedPtr msg);
    void _subscriber_callback_reset(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _publisher_callback();

    rclcpp::QoS _qos = rclcpp::QoS(10);

    Driver driver;

    // 定数
    const int interval_ms;
    const bool is_reversed;
    const double limit_speed;
    const double gear_rate;
    const double pos_limit_min;
    const double pos_limit_max;

    // 変数
    double pos_ref = 0.0;

    // 動作モード
    enum class Mode{
        cmd,
        stay,
        stop
    } mode = Mode::stay;
};

}  // namespace cybergear_interface
