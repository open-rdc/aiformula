#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <mapless_planning_msgs/msg/road_segments.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "road_segments_provider/ipm.hpp"
#include "road_segments_provider/visibility_control.h"

namespace road_segments_provider
{

class RoadSegmentsProviderNode : public rclcpp::Node
{
public:
    ROAD_SEGMENTS_PROVIDER_PUBLIC
    explicit RoadSegmentsProviderNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ROAD_SEGMENTS_PROVIDER_PUBLIC
    explicit RoadSegmentsProviderNode(
        const std::string & name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & lane_mask,
        const sensor_msgs::msg::Image::ConstSharedPtr & da_mask);

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

    // Builds CameraParams from stored intrinsics and TF lookup result.
    // Returns nullopt if TF is unavailable.
    std::optional<CameraParams> build_camera_params();

    // Extracts ego lane left/right boundaries from the lane mask image.
    // Left boundary:  rightmost lane pixels in left half (u < width/2), row by row.
    // Right boundary: leftmost lane pixels in right half (u >= width/2), row by row.
    // Both are returned sorted by increasing ground-x (near to far).
    std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>>
    extract_lane_boundaries(
        const cv::Mat & lane_img,
        const CameraParams & params) const;

    visualization_msgs::msg::MarkerArray build_markers(
        const std::vector<geometry_msgs::msg::Point> & left,
        const std::vector<geometry_msgs::msg::Point> & right,
        const std::string & frame_id,
        const rclcpp::Time & stamp) const;

    // Parameters
    const std::string camera_info_topic_;
    const std::string lane_mask_topic_;
    const std::string da_mask_topic_;
    const std::string road_segments_topic_;
    const std::string camera_frame_id_;
    const std::string base_frame_id_;
    const double min_ground_x_m_;
    const double max_ground_x_m_;
    const bool publish_markers_;

    // Subscribers / sync
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> lane_mask_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> da_mask_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Publishers
    rclcpp::Publisher<mapless_planning_msgs::msg::RoadSegments>::SharedPtr road_segments_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera intrinsics (set once from CameraInfo)
    mutable std::mutex camera_info_mutex_;
    std::optional<std::array<double, 4>> camera_intrinsics_;  // [fx, fy, cx, cy]
};

}  // namespace road_segments_provider
