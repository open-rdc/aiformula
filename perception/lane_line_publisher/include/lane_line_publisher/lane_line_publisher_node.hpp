#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lane_line_publisher/ground_projection.hpp"
#include "lane_line_publisher/visibility_control.h"

namespace lane_line_publisher
{

class LaneLinePublisherNode : public rclcpp::Node
{
public:
    LANE_LINE_PUBLISHER_PUBLIC
    explicit LaneLinePublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LANE_LINE_PUBLISHER_PUBLIC
    explicit LaneLinePublisherNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void lane_mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    sensor_msgs::msg::PointCloud2::UniquePtr make_lane_line_point_cloud(
        const std::vector<Eigen::Vector2d>& base_points,
        const builtin_interfaces::msg::Time& stamp) const;
    visualization_msgs::msg::MarkerArray::UniquePtr make_lane_line_marker_array(
        const std::vector<Eigen::Vector2d>& base_points,
        const builtin_interfaces::msg::Time& stamp) const;

    const uint8_t mask_threshold_;
    const double voxel_grid_size_meter_;
    const GroundProjectionLUT ground_projection_look_up_table_;

    // ノイズ除去の基準に使用
    const int min_component_pixels_=10;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lane_line_points_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_line_marker_publisher_;
};

}
