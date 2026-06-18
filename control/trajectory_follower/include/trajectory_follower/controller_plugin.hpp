#pragma once

#include <memory>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>

namespace trajectory_follower
{

class ControllerPlugin
{
public:
    using SharedPtr = std::shared_ptr<ControllerPlugin>;
    virtual ~ControllerPlugin() = default;

    // Called once after plugin instantiation.
    // logger/clock/params are borrowed from the server node.
    virtual void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) = 0;

    // path_in_base is already transformed to base_link frame by the server.
    // current_velocity is the ego longitudinal speed [m/s] (twist.linear.x).
    // Returns std::nullopt when no command should be published.
    // On success, fills target_pose_out (position only, frame_id/stamp set by server).
    virtual std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path_in_base,
        double current_velocity,
        geometry_msgs::msg::PoseStamped & target_pose_out) = 0;
};

}  // namespace trajectory_follower
