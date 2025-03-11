#include "lane_line_publisher/ll_path_follower_node.hpp"

namespace lane_line_publisher
{

LLPathFollowerNode::LLPathFollowerNode(const rclcpp::NodeOptions & options)
    : Node("ll_path_follower_node", options),
      pid_(100),
      control_interval_ms_(100),  
      is_autonomous_(false)
{
    RCLCPP_INFO(this->get_logger(), "LLPathFollowerNode initializing...");

    // PIDゲインとパラメータの取得
    double kp = this->get_parameter_or("ll_p_gain", 1.0);
    double ki = this->get_parameter_or("ll_i_gain", 0.01);
    double kd = this->get_parameter_or("ll_d_gain", 0.1);
    max_linear_velocity_ = this->get_parameter_or("ll_max_linear_vel", 8.0);
    max_angular_velocity_ = this->get_parameter_or("ll_max_angular_vel", 1.0);
    control_interval_ms_ = this->get_parameter_or("ll_interval_ms", 100);

    // PID制御の初期化
    pid_.gain(kp, ki, kd);

    // 経路データのサブスクライバを作成
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&LLPathFollowerNode::path_callback, this, std::placeholders::_1));

    // 自動運転フラグのサブスクライバを作成
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous", 10, std::bind(&LLPathFollowerNode::autonomous_flag_callback, this, std::placeholders::_1));

    // 速度指令のパブリッシャを作成
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 制御ループのタイマーを設定
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_interval_ms_), std::bind(&LLPathFollowerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "LLPathFollowerNode initialized successfully.");
}

void LLPathFollowerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_points_.clear();

    // 受信した経路データを内部のパスデータに格納
    for (const auto& pose : msg->poses) {
        path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
}

void LLPathFollowerNode::autonomous_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 自動運転モードのフラグを更新
    is_autonomous_ = msg->data;
}

// 角度を[-pi, pi]の範囲に正規化する関数
double LLPathFollowerNode::normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void LLPathFollowerNode::publish_cmd_vel(double linear, double angular)
{
    // 速度指令を生成しパブリッシュ
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_publisher_->publish(cmd_vel);
}

void LLPathFollowerNode::control_loop()
{
    // 自動運転が無効の場合は制御をスキップ
    if (!is_autonomous_) {
        // RCLCPP_WARN(this->get_logger(), "Skipping control loop: autonomous=%d", static_cast<int>(is_autonomous_));
        return;
    }
    
    // 目標点を取得（先読みインデックスを設定）
    size_t lookahead_index = std::min(static_cast<size_t>(5), path_points_.size() - 1);
    double target_x = path_points_[lookahead_index].first;
    double target_y = path_points_[lookahead_index].second;

    // 目標点への角度を計算
    double target_angle = std::atan2(target_y, target_x);
    double heading_error = normalize_angle(target_angle);
    double angular_velocity = pid_.cycle(heading_error);

    // 角速度を制限
    angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
    
    // 速度計画を設定
    vel_planner_.vel(max_linear_velocity_);

    // 速度指令をパブリッシュ
    publish_cmd_vel(max_linear_velocity_, angular_velocity);
}

} // namespace lane_line_publisher
