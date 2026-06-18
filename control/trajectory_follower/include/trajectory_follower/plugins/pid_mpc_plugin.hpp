#pragma once

#include "trajectory_follower/controller_plugin.hpp"
#include "trajectory_follower/mpc/lateral_mpc.hpp"
#include "trajectory_follower/mpc/longitudinal_pid.hpp"

namespace trajectory_follower
{

// 縦(並進)方向の速度制御を PID、横方向を MPC で行う追従制御プラグイン。
// Autoware / pilot.auto の pid_longitudinal_controller + mpc_lateral_controller を
// 本ワークスペースの速度指令型インターフェースへ適合したもの。
class PidMpcPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

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

}  // namespace trajectory_follower
