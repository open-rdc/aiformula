#ifndef OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "obstacle_detector_msgs/msg/object_info_array.hpp"

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
    cv::Scalar upper_red_1{20, 255, 255};  // Upper bound for first red range
    cv::Scalar lower_red_2{170, 50, 20};   // Lower bound for second red range
    cv::Scalar upper_red_2{190, 255, 255}; // Upper bound for second red range

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
    // Camera matrix parameters (matching yolopnav CameraIntrinsics)
    double focal_length_x{184.919};  // fx - scaled from 246.558 × (480/640)
    double focal_length_y{205.324};  // fy - scaled from 246.389 × (300/360)
    double center_point_x{238.759};  // cx - scaled from 318.345 × (480/640)
    double center_point_y{155.492};  // cy - scaled from 186.590 × (300/360)
    // Camera position parameters for ground plane projection
    double camera_height{0.54}; // Height above ground in meters (matching yolopnav)
  };

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::vector<cv::Rect> detect_red_objects(const cv::Mat& image);
  bool pixel_to_robot_point(const cv::Point& pixel, cv::Point3f& robot_point);
  void publish_result_image(const cv::Mat& image, const std::vector<cv::Rect>& bboxes, const std_msgs::msg::Header& header);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<obstacle_detector_msgs::msg::ObjectInfoArray>::SharedPtr obstacle_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pos_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_publisher_;

  std::string image_topic_;
  std::string obstacle_info_topic_;
  std::string obstacle_pos_topic_;
  std::string result_image_topic_;

  RedDetectionParams red_params_;
  CameraParams camera_params_;
  cv::Mat camera_matrix_;
  cv::Mat invert_camera_matrix_;
};

}  // namespace obstacle_detector_ros2

#endif  // OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_