#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "trajectory_follower/controller_plugin.hpp"
#include "trajectory_follower/visibility_control.h"

namespace trajectory_follower
{

class ControllerServer : public rclcpp::Node
{
public:
    TRAJECTORY_FOLLOWER_PUBLIC
    explicit ControllerServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    TRAJECTORY_FOLLOWER_PUBLIC
    explicit ControllerServer(
        const std::string & name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void velocity_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void caster_data_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    void timer_callback();
    void publish_stop_command();

    const int interval_ms_;

    pluginlib::ClassLoader<ControllerPlugin> plugin_loader_;
    ControllerPlugin::SharedPtr plugin_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool autonomous_flag_=false;
    nav_msgs::msg::Path::ConstSharedPtr path_;
    geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr velocity_;
    std_msgs::msg::Float64MultiArray::ConstSharedPtr caster_data_;
    std::optional<steered_drive_msg::msg::SteeredDrive> last_cmd_vel_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr caster_data_subscription_;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
