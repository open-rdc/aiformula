#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <camera_utility/camera_utility.hpp>

#include "lane_line_publisher/visibility_control.h"

namespace lane_line_publisher
{

class VectormapVisualizerNode : public rclcpp::Node
{
public:
    LANE_LINE_PUBLISHER_PUBLIC
    explicit VectormapVisualizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LANE_LINE_PUBLISHER_PUBLIC
    explicit VectormapVisualizerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void vector_map_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    std::optional<camera_utility::CameraIntrinsics> camera_intrinsics_;
    const tf2::Transform base_T_camera_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex vector_map_mutex_;
    visualization_msgs::msg::MarkerArray latest_vector_map_markers_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr vector_map_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vectormap_visualize_publisher_;
};

}
