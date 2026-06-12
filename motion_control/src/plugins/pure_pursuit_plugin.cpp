#include "motion_control/plugins/pure_pursuit_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace motion_control
{

namespace
{
constexpr double DEG2RAD = 0.017453292519943295;
constexpr double EPSILON = 1.0e-6;
}

void PurePursuitPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    linear_max_vel_ = params->get_parameter("linear_max.vel").get_value<double>();
    lookahead_distance_ = params->get_parameter("lookahead_distance").get_value<double>();
    steered_gain_ = params->get_parameter("steered_gain").get_value<double>();
    wheelbase_ = params->get_parameter("wheelbase").get_value<double>();
    steering_max_angle_rad_ =
        params->get_parameter("steering_max.pos").get_value<double>() * DEG2RAD;

    if (linear_max_vel_ <= 0.0 || lookahead_distance_ <= 0.0 ||
        steered_gain_ <= 0.0 || wheelbase_ <= 0.0 || steering_max_angle_rad_ <= 0.0)
    {
        throw std::invalid_argument("PurePursuitPlugin: control parameters are invalid");
    }
}

std::optional<steered_drive_msg::msg::SteeredDrive> PurePursuitPlugin::computeCommand(
    const nav_msgs::msg::Path & path_in_base,
    geometry_msgs::msg::PoseStamped & target_pose_out)
{
    TargetPoint target{0.0, 0.0};
    if (!find_lookahead_target(path_in_base, target)) {
        return std::nullopt;
    }

    const double distance = std::hypot(target.x, target.y);
    if (distance < EPSILON) {
        return std::nullopt;
    }

    const double safe_lookahead = std::max(lookahead_distance_, 1.0e-3);
    const double linear_scale = std::clamp(distance / safe_lookahead, 0.0, 1.0);
    const double linear_velocity =
        std::clamp(linear_max_vel_ * linear_scale, 0.0, linear_max_vel_);

    const double alpha = std::atan2(target.y, target.x);
    const double steer_angle =
        std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance_);
    const double steer_clamped =
        std::clamp(steer_angle * steered_gain_, -steering_max_angle_rad_, steering_max_angle_rad_);

    target_pose_out.pose.position.x = target.x;
    target_pose_out.pose.position.y = target.y;
    target_pose_out.pose.position.z = 0.0;
    target_pose_out.pose.orientation.w = 1.0;

    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = linear_velocity;
    command.steering_angle = steer_clamped;
    return command;
}

bool PurePursuitPlugin::find_lookahead_target(
    const nav_msgs::msg::Path & path,
    TargetPoint & target_out) const
{
    bool found_fallback = false;
    TargetPoint fallback{0.0, 0.0};

    for (const auto & pose : path.poses) {
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        if (x <= 0.0) {
            continue;
        }
        const double distance = std::hypot(x, y);
        fallback = {x, y};
        found_fallback = true;
        if (distance >= lookahead_distance_) {
            target_out = {x, y};
            return true;
        }
    }

    if (found_fallback) {
        target_out = fallback;
        return true;
    }

    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "no forward lookahead target in path");
    return false;
}

}  // namespace motion_control

PLUGINLIB_EXPORT_CLASS(motion_control::PurePursuitPlugin, motion_control::ControllerPlugin)
