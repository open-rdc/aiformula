#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

#include "utilities/utils.hpp"
#include "controller/visibility_control.h"

namespace controller{

class Controller : public rclcpp::Node {
public:
    CONTROLLER_PUBLIC
    explicit Controller(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CONTROLLER_PUBLIC
    explicit Controller(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription_joy;

    void _subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_restart;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_autonomous;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const double linear_max_vel;
    const double angular_max_vel;

    bool is_autonomous = false;
    bool is_emergency = false;

    utils::UpEdge upedge_home;
    utils::UpEdge upedge_capture;
    utils::UpEdge upedge_plus;
    utils::UpEdge upedge_a;
    utils::UpEdge upedge_b;

    enum class Axes{
        L_x,
        L_y,
        R_x,
        R_y,
        left_and_right,
        up_and_down
    };
    enum class Buttons{
        B,
        A,
        Y,
        X,
        L,
        R,
        ZL,
        ZR,
        Minus,
        Plus,
        Lstick,
        Rstick,
        Home,
        Capture
    };

};

}  // namespace controller
