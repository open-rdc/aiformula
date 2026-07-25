#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "steered_drive_msg/msg/steered_drive.hpp"
#include <filesystem>
#include "pfoe_localization/ParticleFilter.hpp"

namespace pfoe_localization
{
class PfoeNode : public rclcpp::Node{
public:
    explicit PfoeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit PfoeNode(const std::string& name_space,  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void featureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
private:

    ParticleFilter pf_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pfoe_driving_publisher_;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_subscription_;

    bool pfoe_enabled_;
    bool pfoe_direct_action_enabled_;
    const std::string nav_cmd_topic_;
    const std::string cmd_vel_topic_;
    const std::string feature_topic_;
    const std::string pfoe_driving_topic_;
    int prediction_range_;

};
}