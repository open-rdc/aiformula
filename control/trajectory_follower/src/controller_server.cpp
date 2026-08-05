#include "trajectory_follower/controller_server.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

#include "trajectory_follower/deadman.hpp"
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
  control_period_ms_(get_parameter("control_period_ms").as_int()),
  input_timeout_s_(get_parameter("input_timeout_s").as_double()),
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
    caster_data_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/caster_data", qos,
        std::bind(&ControllerServer::caster_data_callback, this, std::placeholders::_1));
    command_publisher_ =
        create_publisher<steered_drive_msg::msg::SteeredDrive>("/cmd_vel", qos);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&ControllerServer::timer_callback, this));
}

void ControllerServer::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(get_logger(), "pathメッセージがnullのため無視します");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_path_ = msg;
}

void ControllerServer::publish_stop_command()
{
    steered_drive_msg::msg::SteeredDrive stop_command;
    stop_command.velocity = 0.0;
    stop_command.steering_angle = 0.0;
    command_publisher_->publish(stop_command);
}

void ControllerServer::timer_callback()
{
    nav_msgs::msg::Path::SharedPtr path;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose;
    double current_velocity = 0.0;
    bool autonomous_enabled = false;
    std::optional<double> measured_steer;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        path = latest_path_;
        latest_pose = latest_pose_;
        autonomous_enabled = autonomous_enabled_;
        if (latest_velocity_) {
            current_velocity = latest_velocity_->twist.twist.linear.x;
        }
        if (latest_caster_data_ && latest_caster_data_->data.size() >= 2 &&
            !is_stale(now(), latest_caster_stamp_, input_timeout_s_))
        {
            measured_steer = latest_caster_data_->data[1];
        }
    }

    if (!autonomous_enabled) {
        return;
    }
    if (!path) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "経路を待機中です");
        publish_stop_command();
        return;
    }
    if (path->poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "空の経路を受信しました");
        publish_stop_command();
        return;
    }
    if (is_stale(now(), path->header.stamp, input_timeout_s_)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "経路が古いため停止指令を送信します");
        publish_stop_command();
        return;
    }
    if (path->header.frame_id != "map") {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "未対応のframe_idです: %s", path->header.frame_id.c_str());
        publish_stop_command();
        return;
    }
    if (!latest_pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "自己位置を待機中のため停止指令を送信します");
        publish_stop_command();
        return;
    }
    if (is_stale(now(), latest_pose->header.stamp, input_timeout_s_)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "自己位置が古いため停止指令を送信します");
        publish_stop_command();
        return;
    }

    const nav_msgs::msg::Path path_in_base = transform_path_to_base(*path, *latest_pose);

    if (measured_steer) {
        plugin_->setMeasuredSteer(*measured_steer);
    }

    const auto command = plugin_->computeCommand(path_in_base, current_velocity);
    if (!command) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "前方目標が見つからないため停止指令を送信します");
        publish_stop_command();
        return;
    }

    command_publisher_->publish(*command);
}

void ControllerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(get_logger(), "localization poseメッセージがnullのため無視します");
        return;
    }
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(
            get_logger(), "localization poseのframe_idがmapではありません: %s",
            msg->header.frame_id.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void ControllerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(get_logger(), "velocityメッセージがnullのため無視します");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_ = msg;
}

void ControllerServer::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(get_logger(), "autonomousメッセージがnullのため無視します");
        return;
    }
    bool rising_edge = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rising_edge = msg->data && !autonomous_enabled_;
        autonomous_enabled_ = msg->data;
    }
    if (rising_edge) {
        plugin_->reset();
        RCLCPP_INFO(get_logger(), "自律モード再有効化を検知し、コントローラ状態をリセットしました");
    }
}

void ControllerServer::caster_data_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!msg || msg->data.size() < 2) {
        RCLCPP_WARN(get_logger(), "caster_dataメッセージが不正のため無視します");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_caster_data_ = msg;
    latest_caster_stamp_ = now();
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
