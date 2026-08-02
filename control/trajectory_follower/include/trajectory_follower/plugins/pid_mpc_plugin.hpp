#pragma once

#include "trajectory_follower/controller_plugin.hpp"
#include "trajectory_follower/mpc/lateral_mpc.hpp"
#include "trajectory_follower/mpc/longitudinal_pid.hpp"

namespace trajectory_follower
{

class PidMpcPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    void reset() override;

    void setMeasuredSteer(double steer) override;

    std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path_in_base,
        double current_velocity,
        geometry_msgs::msg::PoseStamped & target_pose_out) override;

private:
    rclcpp::Logger logger_{rclcpp::get_logger("pid_mpc_plugin")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>()};

    LongitudinalPid longitudinal_;
    LateralMpc lateral_;
};

}
