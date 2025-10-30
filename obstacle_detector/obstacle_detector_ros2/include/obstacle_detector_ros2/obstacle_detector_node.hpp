#ifndef OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include "obstacle_detector_msgs/msg/object_info_array.hpp"
#include "obstacle_detector_msgs/msg/rect_array.hpp"

namespace obstacle_detector_ros2
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  struct CameraParams {
    // Camera matrix parameters (matching yolopnav CameraIntrinsics)
    double focal_length_x{183.09494018554688};  // fx - scaled from 246.558 × (480/640)
    double focal_length_y{245.82803344726562};  // fy - scaled from 246.389 × (300/360)
    double center_point_x{240};  // cx - scaled from 318.345 × (480/640)
    double center_point_y{150};  // cy - scaled from 186.590 × (300/360)
    // Camera position parameters for ground plane projection
    double camera_height{0.56}; // Height above ground in meters (matching yolopnav)
  };

private:
  void rect_array_callback(const obstacle_detector_msgs::msg::RectArray::SharedPtr msg);
  bool pixel_to_robot_point(const cv::Point& pixel, cv::Point3f& robot_point);

  rclcpp::Subscription<obstacle_detector_msgs::msg::RectArray>::SharedPtr rect_array_subscription_;
  rclcpp::Publisher<obstacle_detector_msgs::msg::ObjectInfoArray>::SharedPtr obstacle_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pos_publisher_;

  std::string obstacle_pixel_topic_;
  std::string obstacle_info_topic_;
  std::string obstacle_pos_topic_;

  CameraParams camera_params_;
  cv::Mat camera_matrix_;
  cv::Mat invert_camera_matrix_;
};

}  // namespace obstacle_detector_ros2

#endif  // OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_