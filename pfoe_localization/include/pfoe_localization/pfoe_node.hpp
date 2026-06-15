#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};
}