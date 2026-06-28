#include "trajectory_follower/controller_server.hpp"

#include <cmath>
#include <stdexcept>

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
  control_period_ms_(get_parameter("control_period_ms").as_int()),
  plugin_loader_("trajectory_follower", "trajectory_follower::ControllerPlugin")
{
    if (control_period_ms_ <= 0) {
        throw std::invalid_argument("control_period_ms must be greater than 0");
    }

    const auto plugin_name = get_parameter("controller_plugin").as_string();
    if (plugin_name.empty()) {
        throw std::invalid_argument("controller_plugin parameter must not be empty");
    }

    plugin_ = plugin_loader_.createSharedInstance(plugin_name);
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    const rclcpp::QoS qos(10);
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planner/local_path", qos,
        std::bind(&ControllerServer::path_callback, this, std::placeholders::_1));
    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", qos,
            std::bind(&ControllerServer::pose_callback, this, std::placeholders::_1));
    velocity_subscription_ =
        create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vectornav/velocity_body", qos,
            std::bind(&ControllerServer::velocity_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/autonomous", qos,
        std::bind(
            &ControllerServer::autonomous_callback, this, std::placeholders::_1));
    command_publisher_ =
        create_publisher<steered_drive_msg::msg::SteeredDrive>("/cmd_vel", qos);
    target_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("/vectormap_control/target_pose", qos);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&ControllerServer::timer_callback, this));
}

void ControllerServer::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("path message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_path_ = msg;
}

void ControllerServer::timer_callback()
{
    nav_msgs::msg::Path::SharedPtr path;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose;
    double current_velocity = 0.0;
    bool autonomous_enabled = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        path = latest_path_;
        latest_pose = latest_pose_;
        autonomous_enabled = autonomous_enabled_;
        if (latest_velocity_) {
            current_velocity = latest_velocity_->twist.twist.linear.x;
        }
    }

    if (!autonomous_enabled) {
        return;
    }
    if (!path) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for local path");
        return;
    }
    if (path->poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "received empty local path");
        return;
    }
    if (path->header.frame_id == "map" && !latest_pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "waiting for localization pose before tracking map-frame path");
        return;
    }

    nav_msgs::msg::Path path_in_base;
    if (path->header.frame_id == "base_link") {
        path_in_base = *path;
    } else if (path->header.frame_id == "map") {
        path_in_base = transform_path_to_base(*path, *latest_pose);
    } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "unsupported path frame_id: %s", path->header.frame_id.c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    const auto command = plugin_->computeCommand(path_in_base, current_velocity, target_pose);
    if (!command) {
        return;
    }

    command_publisher_->publish(*command);
    target_pose.header.stamp = now();
    target_pose.header.frame_id = "base_link";
    target_pose_publisher_->publish(target_pose);
}

void ControllerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("localization pose message must not be null");
    }
    if (msg->header.frame_id != "map") {
        throw std::runtime_error(
            "localization pose frame_id must be map, got " + msg->header.frame_id);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void ControllerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("velocity message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_ = msg;
}

void ControllerServer::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("autonomous message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    autonomous_enabled_ = msg->data;
}

nav_msgs::msg::Path ControllerServer::transform_path_to_base(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose) const
{
    const double yaw = yaw_from_quaternion(ego_pose.pose.pose.orientation);
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

double ControllerServer::yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}
