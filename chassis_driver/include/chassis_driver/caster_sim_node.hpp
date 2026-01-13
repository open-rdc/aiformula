#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "base/velplanner.hpp"
#include "utilities/position_pid.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "std_msgs/msg/float64.hpp"
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subscription_caster_yaw;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void _publisher_callback();

    void _subscriber_callback_caster_yaw(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_ref_vel;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_caster_data;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_caster;
    
    // 速度計画機
    velplanner::VelPlanner linear_planner;
    velplanner::Limit linear_limit;
    velplanner::VelPlanner angular_planner;
    velplanner::Limit angular_limit;

    // 定数
    int interval_ms;
    double wheel_radius;
    double tread;
    double wheelbase;
    double rotate_ratio;
    bool is_reverse_left;
    bool is_reverse_right;
    double caster_max_angle;
    int caster_max_count;

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
