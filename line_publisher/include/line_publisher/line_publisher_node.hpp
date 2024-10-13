#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "line_publisher/visibility_control.h"

namespace line_publisher{

class LinePublisherNode : public rclcpp::Node{
public:
  LINE_PUBLISHER_PUBLIC
  explicit LinePublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  LINE_PUBLISHER_PUBLIC
  explicit LinePublisherNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void LineDetector();

  const int threshold;
  const int ksize;
  const int min_th;
  const int max_th;

  cv_bridge::CvImagePtr cv_img;
  int detected_line_x = 0;
};

}  // namespace line_publisher
