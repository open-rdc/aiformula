#include "object_detection/pylon_detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_utility/camera_utility.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <cmath>
#include <filesystem>
#include <functional>

namespace object_detection {

PylonDetectorNode::PylonDetectorNode(const rclcpp::NodeOptions& options)
    : PylonDetectorNode("", options) {
}

PylonDetectorNode::PylonDetectorNode(
    const std::string&         name_space,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("pylon_detector_node", name_space, options),
      base_T_camera_(camera_utility::getBaseTCamera(*this)) {
    const std::string engine_path =std::filesystem::path(ament_index_cpp::get_package_share_directory("object_detection")) / "weights" / get_parameter("engine_path").as_string();
    detector_ = std::make_unique<YoloxTensorrt>(engine_path, get_parameter("score_threshold").as_double(), get_parameter("nms_threshold").as_double());
    RCLCPP_INFO(get_logger(), "YOLOXエンジンを読み込みました: %s", engine_path.c_str());

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>("/zed/zed_node/rgb/camera_info", rclcpp::SensorDataQoS().keep_last(1), std::bind(&PylonDetectorNode::camera_info_callback, this, std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", rclcpp::SensorDataQoS().keep_last(1), std::bind(&PylonDetectorNode::image_callback, this, std::placeholders::_1));

    objects_publisher_ = create_publisher<object_detection_msgs::msg::ObjectInfoArray>("/perception/objects", rclcpp::SensorDataQoS().keep_last(1));
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/perception/objects_visualize", rclcpp::QoS(1));
    debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/perception/objects_debug_image", rclcpp::SensorDataQoS().keep_last(1));
}

void PylonDetectorNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (!intrinsics_) {
        intrinsics_ = camera_utility::fromCameraInfo(*msg);
        RCLCPP_INFO(get_logger(), "camera_info(%ux%u)を受信しました", msg->width, msg->height);
    }
}

void PylonDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!intrinsics_) {
        RCLCPP_WARN(get_logger(), "camera_info未受信のため画像をスキップします");
        return;
    }
    
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    const auto detections = detector_->detect(image);
    const auto obstacles  = project_to_ground(detections, image);

    objects_publisher_->publish(make_objects(obstacles, msg->header.stamp));
    marker_publisher_->publish(make_markers(obstacles, msg->header.stamp));

    cv_bridge::CvImage debug_image(msg->header, sensor_msgs::image_encodings::BGR8, image);
    debug_image_publisher_->publish(*debug_image.toImageMsg());

    RCLCPP_DEBUG(this->get_logger(), "障害物検出数: %zu", obstacles.size());
}

std::vector<Obstacle> PylonDetectorNode::project_to_ground(
    const std::vector<Detection>& detections,
    cv::Mat&                      debug_image) const {
    std::vector<Obstacle> obstacles;
    obstacles.reserve(detections.size());
    for (const auto& detection : detections) {
        const cv::Rect2d& box = detection.box;
        // markerの配色をclass id毎に割り振り
        const cv::Scalar box_color = detection.class_id == pylon_class_id_ ? cv::Scalar(0.0, 165.0, 255.0) : cv::Scalar(0.0, 0.0, 255.0);
        cv::rectangle(debug_image, box, box_color, 1);

        // bbox底辺を接地線とみなし、左端・中央・右端を地面へ投影する
        const auto   bottom = static_cast<float>(box.y + box.height);
        tf2::Vector3 center;
        tf2::Vector3 left;
        tf2::Vector3 right;
        const bool   projected =
            camera_utility::pixelToPoint(cv::Point2f(static_cast<float>(box.x + box.width * 0.5), bottom), *intrinsics_, base_T_camera_, center) &&
            camera_utility::pixelToPoint(cv::Point2f(static_cast<float>(box.x), bottom), *intrinsics_, base_T_camera_, left) &&
            camera_utility::pixelToPoint(cv::Point2f(static_cast<float>(box.x + box.width), bottom), *intrinsics_, base_T_camera_, right);
        if (!projected) {
            continue;
        }

        const double width = std::hypot(left.x() - right.x(), left.y() - right.y());
        obstacles.push_back(Obstacle{center.x(), center.y(), width, detection.class_id});

        const double distance = std::hypot(center.x(), center.y());
        cv::putText(
            debug_image, cv::format("%.2fm %.2fm", distance, width),
            cv::Point(static_cast<int>(box.x), static_cast<int>(box.y) - 4),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1);
    }
    return obstacles;
}

object_detection_msgs::msg::ObjectInfoArray PylonDetectorNode::make_objects(
    const std::vector<Obstacle>&         obstacles,
    const builtin_interfaces::msg::Time& stamp) const {
    object_detection_msgs::msg::ObjectInfoArray objects;
    objects.header.stamp    = stamp;
    objects.header.frame_id = "base_link";
    objects.objects.reserve(obstacles.size());
    for (const auto& obstacle : obstacles) {
        object_detection_msgs::msg::ObjectInfo object;
        object.x     = static_cast<float>(obstacle.x);
        object.y     = static_cast<float>(obstacle.y);
        object.width = static_cast<float>(obstacle.width);
        object.id    = static_cast<uint8_t>(obstacle.class_id);
        objects.objects.push_back(object);
    }
    return objects;
}

visualization_msgs::msg::MarkerArray PylonDetectorNode::make_markers(
    const std::vector<Obstacle>&         obstacles,
    const builtin_interfaces::msg::Time& stamp) const {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (size_t i = 0; i < obstacles.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp       = stamp;
        marker.header.frame_id    = "base_link";
        marker.ns                 = "obstacle";
        marker.id                 = static_cast<int>(i);
        marker.type               = visualization_msgs::msg::Marker::CYLINDER;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = obstacles[i].x;
        marker.pose.position.y    = obstacles[i].y;
        marker.pose.position.z    = marker_height_m_ * 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = obstacles[i].width;
        marker.scale.y            = obstacles[i].width;
        marker.scale.z            = marker_height_m_;
        marker.color.r            = 1.0F;
        marker.color.g            = obstacles[i].class_id == pylon_class_id_ ? 0.5F : 0.0F;
        marker.color.b            = 0.0F;
        marker.color.a            = 1.0F;
        markers.markers.push_back(marker);
    }
    return markers;
}

}  // namespace object_detection
