#pragma once

#include <memory>
#include <string>

#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_detector/visibility_control.h"

namespace object_detector
{

class ObjectDetectorNode : public rclcpp::Node
{
public:
    OBJECT_DETECTOR_PUBLIC
    explicit ObjectDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    OBJECT_DETECTOR_PUBLIC
    explicit ObjectDetectorNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_empty(const rclcpp::Time& stamp);

    const std::string map_frame_id_;
    const std::string base_frame_id_;
    const double ground_z_threshold_m_;
    const double voxel_leaf_size_m_;
    const double cluster_tolerance_m_;
    const int min_cluster_size_;
    const int max_cluster_size_;
    const double marker_height_m_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Publisher<object_detection_msgs::msg::ObjectInfoArray>::SharedPtr objects_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace object_detector
