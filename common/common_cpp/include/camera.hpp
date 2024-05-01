#ifndef CAMERA_HPP
#define CAMERA_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Original
#include <get_ros_parameter.hpp>

namespace aiformula {

void getCameraParams(rclcpp::Node* node_ptr, const std::string& camera_name, cv::Mat& camera_matrix,
                     cv::Size* image_size = nullptr);

}  // namespace aiformula

#endif  // CAMERA_HPP
