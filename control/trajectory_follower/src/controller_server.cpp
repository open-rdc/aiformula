#include "trajectory_follower/controller_server.hpp"

#include <optional>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_follower
{
namespace
{
nav_msgs::msg::Path transform_path_to_base(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::TransformStamped & base_T_path)
{
    nav_msgs::msg::Path path_in_base;
    path_in_base.header.frame_id = "base_link";
    path_in_base.header.stamp = path.header.stamp;
    path_in_base.poses.reserve(path.poses.size());

    for (const auto & pose : path.poses) {
        geometry_msgs::msg::PoseStamped pose_base;
        tf2::doTransform(pose, pose_base, base_T_path);
        pose_base.header = path_in_base.header;
        path_in_base.poses.push_back(pose_base);
    }

    return path_in_base;
}
}

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: ControllerServer("", options)
{
}

ControllerServer::ControllerServer(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_server_node", name_space, options),
  interval_ms_(get_parameter("interval_ms").as_int()),
  plugin_loader_("trajectory_follower", "trajectory_follower::ControllerPlugin")
{
    // プラグインのロード
    const auto plugin_name = get_parameter("controller_plugin").as_string();
    plugin_ = plugin_loader_.createSharedInstance(plugin_name);
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planner/local_path", rclcpp::QoS(1).best_effort(),
        std::bind(&ControllerServer::path_callback, this, std::placeholders::_1));
    velocity_subscription_ =
        create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vectornav/velocity_body", rclcpp::QoS(1).best_effort(),
            std::bind(&ControllerServer::velocity_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/autonomous", rclcpp::QoS(10),
        std::bind(
            &ControllerServer::autonomous_callback, this, std::placeholders::_1));
    caster_data_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/caster_data", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ControllerServer::caster_data_callback, this, std::placeholders::_1));
    command_publisher_ =
        create_publisher<steered_drive_msg::msg::SteeredDrive>("/cmd_vel", rclcpp::QoS(10));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(interval_ms_),
        std::bind(&ControllerServer::timer_callback, this));
}

void ControllerServer::autonomous_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data && !autonomous_flag_) {
        plugin_->reset();
        RCLCPP_DEBUG(get_logger(), "自律モード再有効化を検知し、コントローラ状態をリセットしました");
    }
    autonomous_flag_ = msg->data;
}

void ControllerServer::path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    path_ = msg;
}

void ControllerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    velocity_ = msg;
}

void ControllerServer::caster_data_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    caster_data_ = msg;
}

void ControllerServer::timer_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!autonomous_flag_) {
        RCLCPP_DEBUG(this->get_logger(), "自律モード待機中");
        return;
    }
    if (!path_) {
        RCLCPP_DEBUG(this->get_logger(), "経路を待機中");
        return;
    }
    if (!velocity_) {
        RCLCPP_DEBUG(this->get_logger(), "速度を待機中");
        return;
    }
    if (path_->poses.size() < 2) {
        RCLCPP_DEBUG(this->get_logger(), "経路が短すぎます");
        return;
    }
    if (!last_cmd_vel_) {
        publish_stop_command();
    }

    geometry_msgs::msg::TransformStamped base_T_path;
    try {
        base_T_path = tf_buffer_->lookupTransform("base_link", path_->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "base_link <- %s のTFが引けないため停止指令を出します: %s",
            path_->header.frame_id.c_str(), error.what());
        publish_stop_command();
        return;
    }
    const nav_msgs::msg::Path path_in_base = transform_path_to_base(*path_, base_T_path);

    if (caster_data_) {
        plugin_->setMeasuredSteer(caster_data_->data[1]);
    }

    const auto command = plugin_->computeCommand(path_in_base, velocity_->twist.twist.linear.x);
    if (!command) {
        RCLCPP_DEBUG(this->get_logger(), "コマンド計算に失敗しました");
        return;
    }
    last_cmd_vel_ = command;
    command_publisher_->publish(std::make_unique<steered_drive_msg::msg::SteeredDrive>(*command));
}

void ControllerServer::publish_stop_command()
{
    auto stop_command = std::make_unique<steered_drive_msg::msg::SteeredDrive>();
    stop_command->velocity = 0.0;
    stop_command->steering_angle = 0.0;
    command_publisher_->publish(std::move(stop_command));
}

}
