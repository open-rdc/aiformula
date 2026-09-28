#include "trajectory_follower/plugins/pure_pursuit_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

#include "trajectory_follower/speed_path_lookup.hpp"
#include "utilities/utils.hpp"

namespace trajectory_follower
{

namespace
{
constexpr double EPSILON = 1.0e-6;
}

void PurePursuitPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    lookahead_gain_ = params->get_parameter("pure_pursuit.lookahead_gain").get_value<double>();
    lookahead_min_distance_ = params->get_parameter("pure_pursuit.lookahead_min_distance").get_value<double>();
    lookahead_max_distance_ = params->get_parameter("pure_pursuit.lookahead_max_distance").get_value<double>();
    steered_gain_ = params->get_parameter("pure_pursuit.steered_gain").get_value<double>();
    wheelbase_ = params->get_parameter("wheelbase").get_value<double>();
    steering_max_angle_rad_ =
        utils::dtor(params->get_parameter("steering_max.pos").get_value<double>());
    velocity_preview_time_ = params->get_parameter("velocity_preview_time").get_value<double>();

    if (lookahead_min_distance_ <= 0.0 || steered_gain_ <= 0.0 || wheelbase_ <= 0.0 ||
        steering_max_angle_rad_ <= 0.0)
    {
        throw std::invalid_argument("PurePursuitPlugin: control parameters are invalid");
    }
}

std::optional<steered_drive_msg::msg::SteeredDrive> PurePursuitPlugin::computeCommand(
    const speed_path_msgs::msg::SpeedPath & path_in_base,
    double current_velocity)
{
    const double lookahead_distance = std::clamp(
        lookahead_gain_ * current_velocity, lookahead_min_distance_, lookahead_max_distance_);

    TargetPoint target{0.0, 0.0};
    if (!find_lookahead_target(path_in_base, lookahead_distance, target)) {
        return std::nullopt;
    }

    const double distance = std::hypot(target.x, target.y);
    if (distance < EPSILON) {
        return std::nullopt;
    }

    const double linear_velocity =
        preview_point(path_in_base, current_velocity, velocity_preview_time_).linear_velocity;

    const double alpha = std::atan2(target.y, target.x);
    const double steer_angle =
        std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);
    const double steer_clamped =
        std::clamp(steer_angle * steered_gain_, -steering_max_angle_rad_, steering_max_angle_rad_);

    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = linear_velocity;
    command.steering_angle = steer_clamped;
    return command;
}

bool PurePursuitPlugin::find_lookahead_target(
    const speed_path_msgs::msg::SpeedPath & path,
    double lookahead_distance,
    TargetPoint & target_out) const
{
    bool found_fallback = false;
    TargetPoint fallback{0.0, 0.0};

    for (const auto & point : path.points) {
        const double x = point.pose.position.x;
        const double y = point.pose.position.y;
        if (x <= 0.0) {
            continue;
        }
        const double distance = std::hypot(x, y);
        fallback = {x, y};
        found_fallback = true;
        if (distance >= lookahead_distance) {
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

}

PLUGINLIB_EXPORT_CLASS(trajectory_follower::PurePursuitPlugin, trajectory_follower::ControllerPlugin)
