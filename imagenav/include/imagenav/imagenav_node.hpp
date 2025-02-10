#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "imagenav/obstacle_detector.hpp"
#include "imagenav/line_detector.hpp"

#include "utilities/position_pid.hpp"

#include "imagenav/visibility_control.h"

namespace imagenav{

class ImageNav : public rclcpp::Node{
public:
  IMAGENAV_PUBLIC
  explicit ImageNav(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  IMAGENAV_PUBLIC
  explicit ImageNav(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void ImageNavigation(void);

  const int interval_ms;
  const double linear_max_;
  const double angular_max_;
  const bool visualize_flag_;
  bool autonomous_flag_=false;

  int left_line_x=0;
  int right_line_x=0;

  std::vector<cv::Point> center_points;

  controller::PositionPid pid;

  imagenav::LineDetector line;
  // まだ未実装
  imagenav::ObstacleDetector obstacle;
};

}  // namespace imagenav
