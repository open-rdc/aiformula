#include "lane_line_publisher/ll_path_follower_node.hpp"

namespace lane_line_publisher
{

LLPathFollowerNode::LLPathFollowerNode(const rclcpp::NodeOptions & options)
    : Node("ll_path_follower_node", options),
    pid_(100),
    control_interval_ms_(100),
    is_autonomous_(false),
    max_linear_velocity_(1.0),
    max_angular_velocity_(3.14)
    {
    // パラメータの取得
    this->get_parameter_or("max_linear_vel", max_linear_velocity_, max_linear_velocity_);
    this->get_parameter_or("max_angular_vel", max_angular_velocity_, max_angular_velocity_);
    this->get_parameter_or("interval_ms", control_interval_ms_, control_interval_ms_);

    pid_.gain(get_parameter("p_gain").as_double(), get_parameter("i_gain").as_double(), get_parameter("d_gain").as_double());

    // サブスクライバの作成
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&LLPathFollowerNode::path_callback, this, std::placeholders::_1));
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous", 10, std::bind(&LLPathFollowerNode::autonomous_flag_callback, this, std::placeholders::_1));

    // パブリッシャの作成
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 制御ループのタイマーを設定
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_interval_ms_), std::bind(&LLPathFollowerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "PathFollowerNode initialized.");
}

// 経路データのコールバック関数
void LLPathFollowerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_points_.clear();
    for (const geometry_msgs::msg::PoseStamped& pose : msg->poses) {
        path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
}

// 自動運転フラグのコールバック関数
void LLPathFollowerNode::autonomous_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_autonomous_ = msg->data;
}

// 制御ループ
void LLPathFollowerNode::control_loop()
{
    if (!is_autonomous_ || path_points_.empty()) {
        return;
    }

    // 目標点を取得（先読みインデックスを設定）
    size_t lookahead_index = std::min(static_cast<size_t>(5), path_points_.size() - 1);
    double target_x = path_points_[lookahead_index].first;
    double target_y = path_points_[lookahead_index].second;

    // 目標点への角度を計算
    double target_angle = std::atan2(target_y, target_x);
    double heading_error = normalize_angle(target_angle);

    // 角速度を計算
    double angular_velocity = pid_.cycle(heading_error);
    angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);

    // 速度計画を設定
    vel_planner_.vel(max_linear_velocity_);

    // 速度指令をパブリッシュ
    publish_cmd_vel(max_linear_velocity_, angular_velocity);
}

// 角度を[-pi, pi]の範囲に正規化する関数
double LLPathFollowerNode::normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// 速度指令をパブリッシュする関数
void LLPathFollowerNode::publish_cmd_vel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_publisher_->publish(cmd_vel);
}

} // namespace lane_line_publisher