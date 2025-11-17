#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "base/velplanner.hpp"
#include "base/position_pid.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"

#include "chassis_driver/visibility_control.h"

namespace chassis_driver{

class ChassisDriver : public rclcpp::Node {
public:
    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_caster;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_emergency;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_caster(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _publisher_callback();
    void send_rpm(const double linear_vel, const double angular_vel);

    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_ref_vel;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr publisher_odrive;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_caster_data;

    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr odrive_axis_client_;

    rclcpp::QoS _qos = rclcpp::QoS(10);


    // 速度計画機
    velplanner::VelPlanner linear_planner;
    const velplanner::Limit linear_limit;
    velplanner::VelPlanner angular_planner;
    const velplanner::Limit angular_limit;

    // 定数
    const int interval_ms;
    const double wheel_radius;
    const double tread;
    const double wheelbase;
    const double rotate_ratio;
    const bool is_reverse_left;
    const bool is_reverse_right;
    const double caster_max_angle;
    const int caster_max_count;

    // 変数
    double caster_orientation = 0.0;

    // 動作モード
    enum class Mode{
        cmd,
        stay,
        stop
    } mode = Mode::stay;

};

}  // namespace chassis_driver
