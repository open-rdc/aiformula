#include "trajectory_follower/plugins/pure_pursuit_plugin.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

#include "trajectory_follower/speed_path_lookup.hpp"
#include "utilities/utils.hpp"

namespace trajectory_follower
{

void PurePursuitPlugin::initialize(
    const rclcpp::Logger &,
    const rclcpp::Clock::SharedPtr &,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    lookahead_gain_ = params->get_parameter("pure_pursuit.lookahead_gain").as_double();
    lookahead_min_distance_ = params->get_parameter("pure_pursuit.lookahead_min_distance").as_double();
    lookahead_max_distance_ = params->get_parameter("pure_pursuit.lookahead_max_distance").as_double();
    steered_gain_ = params->get_parameter("pure_pursuit.steered_gain").as_double();
    wheelbase_ = params->get_parameter("wheelbase").as_double();
    steering_max_angle_rad_ = utils::dtor(params->get_parameter("steering_max.pos").as_double());
    velocity_preview_time_ = params->get_parameter("velocity_preview_time").as_double();
}

steered_drive_msg::msg::SteeredDrive PurePursuitPlugin::computeCommand(
    const speed_path_msgs::msg::SpeedPath & path_in_base,
    double current_velocity)
{
    // 1. 速度に応じた先読み距離を決める
    const double lookahead_distance = std::clamp(
        lookahead_gain_ * current_velocity, lookahead_min_distance_, lookahead_max_distance_);

    // 2. 前方の点から先読み距離以上の最初の点を目標にする（無ければ最後の前方点）
    double target_x = 0.0;
    double target_y = 0.0;
    for (const auto & point : path_in_base.points) {
        const double x = point.pose.position.x;
        const double y = point.pose.position.y;
        if (x <= 0.0) {
            continue;
        }
        target_x = x;
        target_y = y;
        if (std::hypot(x, y) >= lookahead_distance) {
            break;
        }
    }

    // 3. 目標点への角度から舵角を求める
    const double alpha = std::atan2(target_y, target_x);
    const double steer_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);

    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = preview_point(path_in_base, current_velocity, velocity_preview_time_).linear_velocity;
    command.steering_angle =
        std::clamp(steer_angle * steered_gain_, -steering_max_angle_rad_, steering_max_angle_rad_);
    return command;
}

}

PLUGINLIB_EXPORT_CLASS(trajectory_follower::PurePursuitPlugin, trajectory_follower::ControllerPlugin)
