#include "trajectory_follower/controller_server.hpp"

#include <cmath>
#include <optional>

#include "utilities/utils.hpp"

namespace trajectory_follower
{

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

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planner/local_path", rclcpp::QoS(1).best_effort(),
        std::bind(&ControllerServer::path_callback, this, std::placeholders::_1));
    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", rclcpp::QoS(1).best_effort(),
            std::bind(&ControllerServer::pose_callback, this, std::placeholders::_1));
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

void ControllerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(
            get_logger(), "localization poseのframe_idがmapではありません: %s",
            msg->header.frame_id.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    pose_ = msg;
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
    if (!pose_) {
        RCLCPP_DEBUG(this->get_logger(), "自己位置を待機中");
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
        auto stop_command = std::make_unique<steered_drive_msg::msg::SteeredDrive>();
        stop_command->velocity = 0.0;
        stop_command->steering_angle = 0.0;
        command_publisher_->publish(std::move(stop_command));
    }

    const nav_msgs::msg::Path path_in_base = transform_path_to_base(*path_, *pose_);

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

nav_msgs::msg::Path ControllerServer::transform_path_to_base(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose) const
{
    const double yaw = utils::yaw_from_quaternion(ego_pose.pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double ego_x = ego_pose.pose.pose.position.x;
    const double ego_y = ego_pose.pose.pose.position.y;

    nav_msgs::msg::Path path_in_base;
    path_in_base.header.frame_id = "base_link";
    path_in_base.header.stamp = path.header.stamp;
    path_in_base.poses.reserve(path.poses.size());

    for (const auto & p : path.poses) {
        const double dx = p.pose.position.x - ego_x;
        const double dy = p.pose.position.y - ego_y;
        geometry_msgs::msg::PoseStamped pose_base;
        pose_base.pose.position.x = cos_yaw * dx + sin_yaw * dy;
        pose_base.pose.position.y = -sin_yaw * dx + cos_yaw * dy;
        pose_base.pose.position.z = 0.0;
        pose_base.pose.orientation.w = 1.0;
        path_in_base.poses.push_back(pose_base);
    }

    return path_in_base;
}

}
