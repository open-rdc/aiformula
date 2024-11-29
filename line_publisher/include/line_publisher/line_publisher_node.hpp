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

  rclcpp::TimerBase::SharedPtr timer_;
  cv_bridge::CvImagePtr cv_img;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img) {
    cv_img = cv_bridge::toCvCopy(img, img->encoding);
  }
  void LineDetector();

  const int ksize;
  const int threshold;
  const int min_th;
  const int max_th;
  const int interval_ms;
};

}  // namespace line_publisher
