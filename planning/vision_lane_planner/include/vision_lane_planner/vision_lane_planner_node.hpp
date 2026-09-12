#pragma once

#include "vision_lane_planner/lane_path_builder.hpp"
#include "vision_lane_planner/visibility_control.h"
#include "vision_lane_planner/vision_lane_tensorrt.hpp"

#include <camera_utility/camera_utility.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <string>

namespace vision_lane_planner {

class VisionLanePlannerNode : public rclcpp::Node {
   public:
    VISION_LANE_PLANNER_PUBLIC
    explicit VisionLanePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VISION_LANE_PLANNER_PUBLIC
    explicit VisionLanePlannerNode(
        const std::string&         name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void navigation_command_callback(const std_msgs::msg::String::ConstSharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    visualization_msgs::msg::MarkerArray::UniquePtr make_marker_array(
        const nav_msgs::msg::Path& path) const;
    sensor_msgs::msg::Image::UniquePtr make_debug_image(
        const cv::Mat& source, const SlotPrediction& prediction,
        const std_msgs::msg::Header& header) const;

    const PathBuilderParams path_params_;
    Slot                    commanded_slot_;

    std::unique_ptr<VisionLaneTensorrt>             inference_;
    const tf2::Transform                            base_T_camera_;
    std::optional<camera_utility::CameraIntrinsics> intrinsics_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr      camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           image_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             navigation_command_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  global_path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              debug_image_publisher_;
};

}  // namespace vision_lane_planner
