#pragma once

#include "gnssnav/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

namespace gnssnav{

class Follower : public rclcpp::Node{
public:
    GNSSNAV_PUBLIC
    explicit Follower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    GNSSNAV_PUBLIC
    explicit Follower(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

};

}  // namespace gnssnav
