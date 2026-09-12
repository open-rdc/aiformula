#include "vision_lane_planner/vision_lane_planner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <utility>

namespace vision_lane_planner {

VisionLanePlannerNode::VisionLanePlannerNode(const rclcpp::NodeOptions& options)
    : VisionLanePlannerNode("", options) {
}

VisionLanePlannerNode::VisionLanePlannerNode(
    const std::string&         name_space,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("vision_lane_planner_node", name_space, options),
      path_params_{
          get_parameter("exist_threshold").as_double(),
          get_parameter("valid_threshold").as_double(),
          get_parameter("path_resample_interval_m").as_double(),
          static_cast<std::size_t>(get_parameter("min_path_points").as_int())},
      commanded_slot_(parse_slot(get_parameter("default_navigation_command").as_string())),
      base_T_camera_(camera_utility::getBaseTCamera(*this)) {
    const std::string engine_path =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("vision_lane_planner")) /
        "weights" / get_parameter("engine_path").as_string();
    inference_ = std::make_unique<VisionLaneTensorrt>(engine_path);
    RCLCPP_INFO(get_logger(), "VisionPlannerエンジンを読み込みました: %s", engine_path.c_str());

    camera_info_subscription_        = create_subscription<sensor_msgs::msg::CameraInfo>("/zed/zed_node/rgb/camera_info", rclcpp::SystemDefaultsQoS(), std::bind(&VisionLanePlannerNode::camera_info_callback, this, std::placeholders::_1));
    image_subscription_              = create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", rclcpp::SensorDataQoS().keep_last(1), std::bind(&VisionLanePlannerNode::image_callback, this, std::placeholders::_1));
    navigation_command_subscription_ = create_subscription<std_msgs::msg::String>("/planning/nav_cmd", rclcpp::SensorDataQoS().keep_last(1), std::bind(&VisionLanePlannerNode::navigation_command_callback, this, std::placeholders::_1));

    global_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/global_path", rclcpp::QoS(1).keep_last(1));
    marker_publisher_      = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/vision_lane_visualize", rclcpp::QoS(1));
    debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/planning/vision_lane_debug_image", rclcpp::SensorDataQoS().keep_last(1));
}

void VisionLanePlannerNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (intrinsics_) return;

    intrinsics_ = camera_utility::fromCameraInfo(*msg);
    RCLCPP_INFO(get_logger(), "camera_info(%ux%u)を受信しました", msg->width, msg->height);
}

void VisionLanePlannerNode::navigation_command_callback(
    const std_msgs::msg::String::ConstSharedPtr msg) {
    commanded_slot_ = parse_slot(msg->data);
}

void VisionLanePlannerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!intrinsics_) return;

    const cv::Mat source = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;

    // letterbox処理 360x640 → 384x640
    cv::Mat padded;
    cv::copyMakeBorder(
        source, padded, pad_top, input_height - source_height - pad_top,
        pad_left, input_width - source_width - pad_left, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    const SlotPrediction prediction = inference_->infer(padded);
    if (!prediction.valid) {
        RCLCPP_WARN(get_logger(), "推論に失敗しました: %s", inference_->last_error().c_str());
    }

    const PathResult result = build_path(prediction, commanded_slot_, *intrinsics_, base_T_camera_, path_params_, msg->header.stamp);
    global_path_publisher_->publish(std::make_unique<nav_msgs::msg::Path>(result.path));

    // サブスクライバがいる場合のみ実行
    if (marker_publisher_->get_subscription_count() > 0) {
        marker_publisher_->publish(make_marker_array(result.path));
    }
    if (debug_image_publisher_->get_subscription_count() > 0) {
        debug_image_publisher_->publish(make_debug_image(source, prediction, msg->header));
    }
}

visualization_msgs::msg::MarkerArray::UniquePtr VisionLanePlannerNode::make_marker_array(
    const nav_msgs::msg::Path& path) const {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = path.header;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(delete_marker);

    visualization_msgs::msg::Marker line_marker;
    line_marker.header             = path.header;
    line_marker.ns                 = "vision_lane";
    line_marker.id                 = 0;
    line_marker.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action             = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x            = 0.08;
    line_marker.color.r            = 0.0F;
    line_marker.color.g            = 0.9F;
    line_marker.color.b            = 0.2F;
    line_marker.color.a            = 1.0F;
    line_marker.points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
        line_marker.points.push_back(pose.pose.position);
    }

    marker_array->markers.push_back(std::move(line_marker));
    return marker_array;
}

sensor_msgs::msg::Image::UniquePtr VisionLanePlannerNode::make_debug_image(
    const cv::Mat&               source,
    const SlotPrediction&        prediction,
    const std_msgs::msg::Header& header) const {
    static const std::array<cv::Scalar, num_slots> slot_colors{
        cv::Scalar(0, 220, 0), cv::Scalar(0, 120, 230), cv::Scalar(255, 140, 0)};

    cv::Mat canvas = source.clone();
    for (std::size_t slot = 0; slot < num_slots; ++slot) {
        for (std::size_t row = 0; row < num_rows; ++row) {
            if (1.0 / (1.0 + std::exp(-static_cast<double>(prediction.valid_logit[slot][row]))) <= path_params_.valid_threshold) {
                continue;
            }
            const double u = static_cast<double>(prediction.position[slot][row]) * (input_width - 1) - pad_left;
            const double v = row_anchor_v(row) - pad_top;
            if (u < 0.0 || u > canvas.cols - 1 || v < 0.0 || v > canvas.rows - 1) {
                continue;
            }
            cv::circle(canvas, cv::Point(static_cast<int>(u), static_cast<int>(v)), 3, slot_colors[slot], -1);
        }
    }

    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, canvas).toImageMsg(*message);
    return message;
}

}  // namespace vision_lane_planner
