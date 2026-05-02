#pragma once

#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>

#include "vectormap_control/visibility_control.h"

namespace vectormap_control
{

class VectormapPurePursuitNode : public rclcpp::Node
{
public:
    VECTORMAP_CONTROL_PUBLIC
    explicit VectormapPurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VECTORMAP_CONTROL_PUBLIC
    explicit VectormapPurePursuitNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct TargetPoint
    {
        double x;
        double y;
    };

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    bool path_pose_to_base_point(
        const geometry_msgs::msg::PoseStamped& path_pose,
        const geometry_msgs::msg::PoseWithCovarianceStamped& ego_pose,
        const std::string& path_frame_id,
        TargetPoint& point_out);
    bool find_lookahead_target(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::PoseWithCovarianceStamped* ego_pose,
        TargetPoint& target_out);
    steered_drive_msg::msg::SteeredDrive make_command(const TargetPoint& target) const;
    void publish_target_pose(const TargetPoint& target) const;

    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion);

    const std::string path_topic_;
    const std::string pose_topic_;
    const std::string autonomous_topic_;
    const std::string cmd_vel_topic_;
    const std::string target_pose_topic_;
    const std::string map_frame_id_;
    const std::string base_frame_id_;
    const double linear_max_vel_;
    const double lookahead_distance_;
    const double steered_gain_;
    const double wheelbase_;
    const double steering_max_angle_rad_;
    const rclcpp::QoS qos_;

    bool autonomous_enabled_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
};

}  // namespace vectormap_control
