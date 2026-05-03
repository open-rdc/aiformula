#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "steered_drive_msg/msg/steered_drive.hpp"

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

    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_restart;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_autonomous;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const double linear_max_vel;
    const double steering_max_angle;

    bool is_autonomous = false;

    utils::UpEdge upedge_share;
    utils::UpEdge upedge_options;
    utils::UpEdge upedge_ps;

    enum class Axes{
        L_x,
        L_y,
        R_x,
        R_y,
        L2,
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
