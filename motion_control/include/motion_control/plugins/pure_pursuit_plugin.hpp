#pragma once

#include "motion_control/controller_plugin.hpp"

namespace motion_control
{

class PurePursuitPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseWithCovarianceStamped * ego_pose,
        geometry_msgs::msg::PoseStamped & target_pose_out) override;

private:
    struct TargetPoint
    {
        double x;
        double y;
    };

    bool find_lookahead_target(const nav_msgs::msg::Path & path, TargetPoint & target_out) const;

    static nav_msgs::msg::Path transform_to_base(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose);
    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);

    rclcpp::Logger logger_{rclcpp::get_logger("pure_pursuit_plugin")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>()};
    double linear_max_vel_{0.0};
    double lookahead_distance_{0.0};
    double steered_gain_{0.0};
    double wheelbase_{0.0};
    double steering_max_angle_rad_{0.0};
};

}  // namespace motion_control
