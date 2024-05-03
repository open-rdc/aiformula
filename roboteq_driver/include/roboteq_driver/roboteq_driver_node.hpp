#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

#include "roboteq_driver/visibility_control.h"

namespace roboteq_driver{

class RoboteqDriver : public rclcpp::Node {
public:
    ROBOTEQ_DRIVER_PUBLIC
    explicit RoboteqDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ROBOTEQ_DRIVER_PUBLIC
    explicit RoboteqDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_vel;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_rpm;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_emergency;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_vel(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_rpm(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _publisher_callback();
    void send_rpm(const double linear_vel, const double angular_vel);

    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    // 定数
    const int interval_ms;
    const double wheel_radius;
    const double tread;
    const double rotate_ratio;
    const bool is_reverse_left;
    const bool is_reverse_right;

    // 制御
    std::shared_ptr<geometry_msgs::msg::Vector3> vel;

    // 動作モード
    enum class Mode{
        cmd,
        stay,
        stop
    } mode = Mode::stay;

};

}  // namespace roboteq_driver
