#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vectormap_msgs/msg/vector_map.hpp"
#include "vectormap_server/visibility_control.h"

namespace vectormap_server
{

class VectormapServerNode : public rclcpp::Node
{
public:
    VECTORMAP_SERVER_PUBLIC
    explicit VectormapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VECTORMAP_SERVER_PUBLIC
    explicit VectormapServerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    static std::string resolve_map_path(const std::string& map_path);

    geometry_msgs::msg::TransformStamped create_earth_to_map_transform() const;
    void publish_static_transforms();
    void publish_callback();

    const std::string map_path_;
    const int64_t publish_period_ms_;
    const double map_yaw_from_east_;
    const rclcpp::QoS qos_;

    vectormap_msgs::msg::VectorMap map_msg_;
    visualization_msgs::msg::MarkerArray marker_array_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
