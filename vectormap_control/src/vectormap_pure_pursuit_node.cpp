#include "vectormap_control/vectormap_pure_pursuit_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vectormap_control
{
namespace
{
constexpr double DEG2RAD = 0.017453292519943295;
constexpr double EPSILON = 1.0e-6;
}

VectormapPurePursuitNode::VectormapPurePursuitNode(const rclcpp::NodeOptions& options)
: VectormapPurePursuitNode("", options)
{
}

VectormapPurePursuitNode::VectormapPurePursuitNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("vectormap_pure_pursuit_node", name_space, options),
  path_topic_(get_parameter("path_topic").as_string()),
  pose_topic_(get_parameter("pose_topic").as_string()),
  autonomous_topic_(get_parameter("autonomous_topic").as_string()),
  cmd_vel_topic_(get_parameter("cmd_vel_topic").as_string()),
  target_pose_topic_(get_parameter("target_pose_topic").as_string()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  linear_max_vel_(get_parameter("linear_max.vel").as_double()),
  lookahead_distance_(get_parameter("lookahead_distance").as_double()),
  steered_gain_(get_parameter("steered_gain").as_double()),
  wheelbase_(get_parameter("wheelbase").as_double()),
  steering_max_angle_rad_(get_parameter("steering_max.pos").as_double() * DEG2RAD),
  qos_(rclcpp::QoS(10)),
  autonomous_enabled_(false)
{
    if (path_topic_.empty() || pose_topic_.empty() || autonomous_topic_.empty() ||
        cmd_vel_topic_.empty() || target_pose_topic_.empty())
    {
        throw std::invalid_argument("vectormap pure pursuit topic parameters must not be empty");
    }
    if (map_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("vectormap pure pursuit frame parameters must not be empty");
    }
    if (linear_max_vel_ <= 0.0 ||
        lookahead_distance_ <= 0.0 ||
        steered_gain_ <= 0.0 ||
        wheelbase_ <= 0.0 ||
        steering_max_angle_rad_ <= 0.0)
    {
        throw std::invalid_argument("vectormap pure pursuit control parameters are invalid");
    }

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_,
        qos_,
        std::bind(&VectormapPurePursuitNode::path_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_,
        qos_,
        std::bind(&VectormapPurePursuitNode::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>(
        autonomous_topic_,
        qos_,
        std::bind(&VectormapPurePursuitNode::autonomous_callback, this, std::placeholders::_1));
    command_publisher_ = create_publisher<steered_drive_msg::msg::SteeredDrive>(cmd_vel_topic_, qos_);
    target_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_pose_topic_, qos_);
}

void VectormapPurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("local path message must not be null");
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
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "received empty local path");
        return;
    }

    const geometry_msgs::msg::PoseWithCovarianceStamped* ego_pose =
        latest_pose ? latest_pose.get() : nullptr;
    if (msg->header.frame_id == map_frame_id_ && ego_pose == nullptr) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "waiting for localization pose before tracking map-frame path");
        return;
    }

    TargetPoint target{0.0, 0.0};
    if (!find_lookahead_target(*msg, ego_pose, target)) {
        return;
    }

    const auto command = make_command(target);
    command_publisher_->publish(command);
    publish_target_pose(target);
}

void VectormapPurePursuitNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("localization pose message must not be null");
    }
    if (msg->header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "localization pose frame_id must be " + map_frame_id_ + ", got " + msg->header.frame_id);
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void VectormapPurePursuitNode::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("autonomous message must not be null");
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        autonomous_enabled_ = msg->data;
    }
}

bool VectormapPurePursuitNode::path_pose_to_base_point(
    const geometry_msgs::msg::PoseStamped& path_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped& ego_pose,
    const std::string& path_frame_id,
    TargetPoint& point_out)
{
    if (path_frame_id == base_frame_id_) {
        point_out.x = path_pose.pose.position.x;
        point_out.y = path_pose.pose.position.y;
        return true;
    }
    if (path_frame_id != map_frame_id_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "unsupported local path frame_id: %s",
            path_frame_id.c_str());
        return false;
    }

    const double yaw = yaw_from_quaternion(ego_pose.pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double dx = path_pose.pose.position.x - ego_pose.pose.pose.position.x;
    const double dy = path_pose.pose.position.y - ego_pose.pose.pose.position.y;
    point_out.x = cos_yaw * dx + sin_yaw * dy;
    point_out.y = -sin_yaw * dx + cos_yaw * dy;
    return true;
}

bool VectormapPurePursuitNode::find_lookahead_target(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::PoseWithCovarianceStamped* const ego_pose,
    TargetPoint& target_out)
{
    if (path.header.frame_id != base_frame_id_ && ego_pose == nullptr) {
        return false;
    }

    bool found_fallback = false;
    TargetPoint fallback{0.0, 0.0};
    for (const auto& pose : path.poses) {
        TargetPoint point{0.0, 0.0};
        if (path.header.frame_id == base_frame_id_) {
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
        } else if (!path_pose_to_base_point(pose, *ego_pose, path.header.frame_id, point)) {
            return false;
        }

        if (point.x <= 0.0) {
            continue;
        }
        const double distance = std::hypot(point.x, point.y);
        fallback = point;
        found_fallback = true;
        if (distance >= lookahead_distance_) {
            target_out = point;
            return true;
        }
    }

    if (found_fallback) {
        target_out = fallback;
        return true;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "local path has no forward lookahead target");
    return false;
}

steered_drive_msg::msg::SteeredDrive VectormapPurePursuitNode::make_command(
    const TargetPoint& target) const
{
    steered_drive_msg::msg::SteeredDrive command;
    const double distance = std::hypot(target.x, target.y);
    if (distance < EPSILON) {
        return command;
    }

    const double safe_lookahead = std::max(lookahead_distance_, 1.0e-3);
    const double linear_scale = std::clamp(distance / safe_lookahead, 0.0, 1.0);
    const double linear_velocity = std::clamp(linear_max_vel_ * linear_scale, 0.0, linear_max_vel_);

    const double alpha = std::atan2(target.y, target.x);
    const double steer_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance_);
    const double steer_angle_clamped =
        std::clamp(steer_angle * steered_gain_, -steering_max_angle_rad_, steering_max_angle_rad_);

    command.velocity = linear_velocity;
    command.steering_angle = steer_angle_clamped;
    return command;
}

void VectormapPurePursuitNode::publish_target_pose(const TargetPoint& target) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = base_frame_id_;
    pose.pose.position.x = target.x;
    pose.pose.position.y = target.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    target_pose_publisher_->publish(pose);
}

double VectormapPurePursuitNode::yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& quaternion)
{
    const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace vectormap_control
