#include "motion_control/controller_server.hpp"

#include <stdexcept>

namespace motion_control
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: ControllerServer("", options)
{
}

ControllerServer::ControllerServer(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_server_node", name_space, options),
  path_topic_(get_parameter("path_topic").as_string()),
  pose_topic_(get_parameter("pose_topic").as_string()),
  autonomous_topic_(get_parameter("autonomous_topic").as_string()),
  cmd_vel_topic_(get_parameter("cmd_vel_topic").as_string()),
  target_pose_topic_(get_parameter("target_pose_topic").as_string()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  plugin_loader_("motion_control", "motion_control::ControllerPlugin")
{
    if (path_topic_.empty() || pose_topic_.empty() || autonomous_topic_.empty() ||
        cmd_vel_topic_.empty() || target_pose_topic_.empty())
    {
        throw std::invalid_argument("controller server topic parameters must not be empty");
    }
    if (map_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("controller server frame parameters must not be empty");
    }

    const auto plugin_name = get_parameter("controller_plugin").as_string();
    if (plugin_name.empty()) {
        throw std::invalid_argument("controller_plugin parameter must not be empty");
    }

    plugin_ = plugin_loader_.createSharedInstance(plugin_name);
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    const rclcpp::QoS qos(10);
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, qos,
        std::bind(&ControllerServer::path_callback, this, std::placeholders::_1));
    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, qos,
            std::bind(&ControllerServer::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>(
        autonomous_topic_, qos,
        std::bind(
            &ControllerServer::autonomous_callback, this, std::placeholders::_1));
    command_publisher_ =
        create_publisher<steered_drive_msg::msg::SteeredDrive>(cmd_vel_topic_, qos);
    target_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, qos);

    const double control_period = get_parameter("control_period_s").as_double();
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period),
        std::bind(&ControllerServer::control_loop, this));
}

void ControllerServer::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("path message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_path_ = msg;
}

void ControllerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("localization pose message must not be null");
    }
    if (msg->header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "localization pose frame_id must be " + map_frame_id_ +
            ", got " + msg->header.frame_id);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void ControllerServer::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("autonomous message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    autonomous_enabled_ = msg->data;
}

void ControllerServer::control_loop()
{
    nav_msgs::msg::Path::SharedPtr path;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose;
    bool autonomous;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        autonomous = autonomous_enabled_;
        path = latest_path_;
        pose = latest_pose_;
    }

    if (!autonomous || !path || path->poses.empty()) return;
    if (path->header.frame_id == map_frame_id_ && !pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "waiting for localization pose before tracking map-frame path");
        return;
    }
    if (path->header.frame_id != map_frame_id_ && path->header.frame_id != base_frame_id_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "unsupported path frame_id: %s", path->header.frame_id.c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    const auto command = plugin_->computeCommand(*path, pose.get(), target_pose);
    if (!command) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "computeCommand returned no command");
        return;
    }

    command_publisher_->publish(*command);
    target_pose.header.stamp = now();
    target_pose.header.frame_id = base_frame_id_;
    target_pose_publisher_->publish(target_pose);
}

}  // namespace motion_control
