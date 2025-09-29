#ifndef OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "obstacle_detector_msgs/msg/object_info_array.hpp"
#include "obstacle_detector_msgs/msg/rect_array.hpp"

namespace obstacle_detector_ros2
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Tunable parameters in header for easy adjustment
  struct RedDetectionParams {
    // HSV color range for red detection
    cv::Scalar lower_red_1{0, 50, 20};     // Lower bound for first red range
    cv::Scalar upper_red_1{10, 255, 255};  // Upper bound for first red range
    cv::Scalar lower_red_2{170, 50, 20};   // Lower bound for second red range
    cv::Scalar upper_red_2{180, 255, 255}; // Upper bound for second red range

    // Morphological operations
    int erode_kernel_size{3};
    int dilate_kernel_size{3};
    int erode_iterations{1};
    int dilate_iterations{2};

    // Contour filtering
    double min_contour_area{15.0};
    double max_contour_area{50000.0};

    // Object tracking
    double default_confidence{0.9};
  };

  struct CameraParams {
    // Camera matrix parameters
    double focal_length_x{800.0};
    double focal_length_y{800.0};
    double center_point_x{240.0};
    double center_point_y{150.0};
    // Camera position parameters for ground plane projection
    double camera_height{0.55}; // Height above ground in meters
  };

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::vector<cv::Rect> detect_red_objects(const cv::Mat& image);
  bool pixel_to_robot_point(const cv::Point& pixel, cv::Point3f& robot_point);
  void publish_result_image(const cv::Mat& image, const std::vector<cv::Rect>& bboxes, const std_msgs::msg::Header& header);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<obstacle_detector_msgs::msg::ObjectInfoArray>::SharedPtr obstacle_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_publisher_;

  std::string image_topic_;
  std::string obstacle_info_topic_;
  std::string result_image_topic_;

  RedDetectionParams red_params_;
  CameraParams camera_params_;
  cv::Mat camera_matrix_;
  cv::Mat invert_camera_matrix_;
};

}  // namespace obstacle_detector_ros2

#endif  // OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_