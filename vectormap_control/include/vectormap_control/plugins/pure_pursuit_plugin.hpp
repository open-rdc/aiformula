#pragma once

#include "vectormap_control/controller_plugin.hpp"

namespace vectormap_control
{

class PurePursuitPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path_in_base,
        geometry_msgs::msg::PoseStamped & target_pose_out) override;

private:
    struct TargetPoint
    {
        double x;
        double y;
    };

    bool find_lookahead_target(const nav_msgs::msg::Path & path, TargetPoint & target_out) const;

    rclcpp::Logger logger_{rclcpp::get_logger("pure_pursuit_plugin")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>()};
    double linear_max_vel_{0.0};
    double lookahead_distance_{0.0};
    double steered_gain_{0.0};
    double wheelbase_{0.0};
    double steering_max_angle_rad_{0.0};
};

}  // namespace vectormap_control
