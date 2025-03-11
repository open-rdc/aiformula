#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include "utilities/visibility_control.h"
#include "utilities/position_pid.hpp"
#include "utilities/velplanner.hpp"
#include "utilities/utils.hpp"
#include "utilities/data_utils.hpp"

namespace lane_line_publisher
{

class LLPathFollowerNode : public rclcpp::Node
{
public:
    UTILITIES_PUBLIC explicit LLPathFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // PIDコントローラ
    controller::PositionPid pid_;  
    velplanner::VelPlanner vel_planner_;

    int control_interval_ms_;
    bool is_autonomous_;

    // サブスクライバ
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;

    // パブリッシャ
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 経路データ
    std::vector<std::pair<double, double>> path_points_;

    // パラメータ
    double max_linear_velocity_;
    double max_angular_velocity_;

    // コールバック関数
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void autonomous_flag_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void control_loop();

    // ユーティリティ関数
    double normalize_angle(double angle);
    void publish_cmd_vel(double linear, double angular);
};

} // namespace lane_line_publisher
