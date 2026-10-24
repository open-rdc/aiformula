#pragma once

#include "trajectory_follower/controller_plugin.hpp"

namespace trajectory_follower
{

class PurePursuitPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger &,
        const rclcpp::Clock::SharedPtr &,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    steered_drive_msg::msg::SteeredDrive computeCommand(
        const speed_path_msgs::msg::SpeedPath & path_in_base,
        double current_velocity) override;

private:
    double lookahead_gain_{0.0};
    double lookahead_min_distance_{0.0};
    double lookahead_max_distance_{0.0};
    double steered_gain_{0.0};
    double wheelbase_{0.0};
    double steering_max_angle_rad_{0.0};
    double velocity_preview_time_{0.0};
};

}
