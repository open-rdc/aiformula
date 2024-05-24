#include "gnssnav/follower_node.hpp"

namespace gnssnav{

Follower::Follower(const rclcpp::NodeOptions& options) : Follower("", options) {}

Follower::Follower(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("gnssnav_follower_node", name_space, options)
{
    
}


}  // namespace gnssnav
