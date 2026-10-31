#pragma once

#include "trajectory_follower/controller_plugin.hpp"
#include "trajectory_follower/pid_mpc/lateral_mpc.hpp"
#include "trajectory_follower/pid_mpc/longitudinal_pid.hpp"

namespace trajectory_follower
{

class PidMpcPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger &,
        const rclcpp::Clock::SharedPtr &,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    void reset() override;

    void setMeasuredSteer(double steer) override;

    steered_drive_msg::msg::SteeredDrive computeCommand(
        const speed_path_msgs::msg::SpeedPath & path_in_base,
        double current_velocity) override;

private:
    LongitudinalPid longitudinal_;
    LateralMpc lateral_;
    double velocity_preview_time_{0.0};
};

}
