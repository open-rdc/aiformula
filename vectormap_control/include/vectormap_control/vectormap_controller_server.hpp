#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>

#include "vectormap_control/controller_plugin.hpp"
#include "vectormap_control/visibility_control.h"

namespace vectormap_control
{

class VectormapControllerServer : public rclcpp::Node
{
public:
    VECTORMAP_CONTROL_PUBLIC
    explicit VectormapControllerServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    VECTORMAP_CONTROL_PUBLIC
    explicit VectormapControllerServer(
        const std::string & name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    nav_msgs::msg::Path transform_path_to_base(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose) const;

    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);

    const std::string path_topic_;
    const std::string pose_topic_;
    const std::string autonomous_topic_;
    const std::string cmd_vel_topic_;
    const std::string target_pose_topic_;
    const std::string map_frame_id_;
    const std::string base_frame_id_;

    pluginlib::ClassLoader<ControllerPlugin> plugin_loader_;
    ControllerPlugin::SharedPtr plugin_;

    bool autonomous_enabled_{false};
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
};

}  // namespace vectormap_control
