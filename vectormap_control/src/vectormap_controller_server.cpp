#include "vectormap_control/vectormap_controller_server.hpp"

#include <cmath>
#include <stdexcept>

namespace vectormap_control
{

VectormapControllerServer::VectormapControllerServer(const rclcpp::NodeOptions & options)
: VectormapControllerServer("", options)
{
}

VectormapControllerServer::VectormapControllerServer(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("vectormap_controller_server_node", name_space, options),
  path_topic_(get_parameter("path_topic").as_string()),
  pose_topic_(get_parameter("pose_topic").as_string()),
  autonomous_topic_(get_parameter("autonomous_topic").as_string()),
  cmd_vel_topic_(get_parameter("cmd_vel_topic").as_string()),
  target_pose_topic_(get_parameter("target_pose_topic").as_string()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  plugin_loader_("vectormap_control", "vectormap_control::ControllerPlugin")
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
        std::bind(&VectormapControllerServer::path_callback, this, std::placeholders::_1));
    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, qos,
            std::bind(&VectormapControllerServer::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>(
        autonomous_topic_, qos,
        std::bind(
            &VectormapControllerServer::autonomous_callback, this, std::placeholders::_1));
    command_publisher_ =
        create_publisher<steered_drive_msg::msg::SteeredDrive>(cmd_vel_topic_, qos);
    target_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, qos);
}

void VectormapControllerServer::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("path message must not be null");
    }

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose;
    bool autonomous_enabled = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_pose = latest_pose_;
        autonomous_enabled = autonomous_enabled_;
    }

    if (!autonomous_enabled) {
        return;
    }
    if (msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "received empty local path");
        return;
    }
    if (msg->header.frame_id == map_frame_id_ && !latest_pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "waiting for localization pose before tracking map-frame path");
        return;
    }

    nav_msgs::msg::Path path_in_base;
    if (msg->header.frame_id == base_frame_id_) {
        path_in_base = *msg;
    } else if (msg->header.frame_id == map_frame_id_) {
        path_in_base = transform_path_to_base(*msg, *latest_pose);
    } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "unsupported path frame_id: %s", msg->header.frame_id.c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    const auto command = plugin_->computeCommand(path_in_base, target_pose);
    if (!command) {
        return;
    }

    command_publisher_->publish(*command);
    target_pose.header.stamp = now();
    target_pose.header.frame_id = base_frame_id_;
    target_pose_publisher_->publish(target_pose);
}

void VectormapControllerServer::pose_callback(
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

void VectormapControllerServer::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("autonomous message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    autonomous_enabled_ = msg->data;
}

nav_msgs::msg::Path VectormapControllerServer::transform_path_to_base(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose) const
{
    const double yaw = yaw_from_quaternion(ego_pose.pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double ego_x = ego_pose.pose.pose.position.x;
    const double ego_y = ego_pose.pose.pose.position.y;

    nav_msgs::msg::Path path_in_base;
    path_in_base.header.frame_id = base_frame_id_;
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

double VectormapControllerServer::yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace vectormap_control
