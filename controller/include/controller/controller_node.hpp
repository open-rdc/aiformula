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
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_nav_start;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const double linear_max_vel;
    const double angular_max_vel;

    bool is_autonomous = false;
    bool is_emergency = false;

    utils::UpEdge upedge_ps;
    utils::UpEdge upedge_share;
    utils::UpEdge upedge_options;
    utils::UpEdge upedge_circle;
    utils::UpEdge upedge_cross;

    enum class Axes{
        L_x,
        L_y,
        L2,
        R_x,
        R_y,
        R2,
        left_and_right,
        up_and_down
    };
    enum class Buttons{
        Cross,
        Circle,
        Triangle,
        Rectangles,
        L1,
        R1,
        L2,
        R2,
        Share,
        Options,
        PS,
        L3,
        R3
    };

};

}  // namespace controller