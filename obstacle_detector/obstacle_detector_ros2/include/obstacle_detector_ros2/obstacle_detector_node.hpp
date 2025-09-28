#ifndef OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "obstacle_detector_msgs/msg/object_info_array.hpp"

namespace obstacle_detector_ros2
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<obstacle_detector_msgs::msg::ObjectInfoArray>::SharedPtr object_info_publisher_;

  std::string image_topic_;
  std::string output_topic_;
};

}  // namespace obstacle_detector_ros2

#endif  // OBSTACLE_DETECTOR_ROS2__OBSTACLE_DETECTOR_NODE_HPP_