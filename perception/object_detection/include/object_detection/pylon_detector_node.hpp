#pragma once

#include "object_detection/visibility_control.h"
#include "object_detection/yolox_tensorrt.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <camera_utility/camera_intrinsics.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <object_detection_msgs/msg/object_info_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <string>
#include <vector>

namespace object_detection {

struct Obstacle {
    double x;
    double y;
    double width;
    int    class_id;
};

class PylonDetectorNode : public rclcpp::Node {
   public:
    OBJECT_DETECTION_PUBLIC
    explicit PylonDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    OBJECT_DETECTION_PUBLIC
    explicit PylonDetectorNode(
        const std::string&         name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::vector<Obstacle> project_to_ground(
        const std::vector<Detection>& detections,
        cv::Mat&                      debug_image) const;

    object_detection_msgs::msg::ObjectInfoArray make_objects(
        const std::vector<Obstacle>&         obstacles,
        const builtin_interfaces::msg::Time& stamp) const;
    visualization_msgs::msg::MarkerArray make_markers(
        const std::vector<Obstacle>&         obstacles,
        const builtin_interfaces::msg::Time& stamp) const;

    const camera_utility::CameraIntrinsics intrinsics_;
    const tf2::Transform                   base_T_camera_;

    static constexpr int    pylon_class_id_            = 0;
    static constexpr int    dynamic_obstacle_class_id_ = 1;
    static constexpr double marker_height_m_           = 0.4;

    std::unique_ptr<YoloxTensorrt> detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                  image_subscription_;
    rclcpp::Publisher<object_detection_msgs::msg::ObjectInfoArray>::SharedPtr objects_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr        marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                     debug_image_publisher_;
};

}  // namespace object_detection
