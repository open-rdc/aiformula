#pragma once

#include <rclcpp/rclcpp.hpp>
#include <topomap_msgs/msg/topo_map.hpp>

#include "topomap_server/visibility_control.h"

namespace topomap_server
{

class TopomapServerNode : public rclcpp::Node
{
public:
    TOPOMAP_SERVER_PUBLIC
    explicit TopomapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Publisher<topomap_msgs::msg::TopoMap>::SharedPtr topomap_publisher_;
};

}
