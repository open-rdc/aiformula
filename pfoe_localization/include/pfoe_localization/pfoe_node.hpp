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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr pub_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};
}