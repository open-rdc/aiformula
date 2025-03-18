#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::vector<geometry_msgs::msg::Point> screenToCamera(const std::vector<cv::Point2f>& cv_points, const cv::Mat& depth_img);
  std::vector<geometry_msgs::msg::Point> interpolateSpline(const std::vector<geometry_msgs::msg::Point>& center_points, int num_point);
  std::vector<geometry_msgs::msg::Point> generatePathPoints(const std::vector<geometry_msgs::msg::Point>& center_points, const std::vector<geometry_msgs::msg::Point>& obstacle_points);
  nav_msgs::msg::Path generatePath(const cv::Mat& cv_img, const cv::Mat& cv_depth_img);
  void publishPoints(const std::vector<geometry_msgs::msg::Point>& points, const std::string& frame_id = "map", float r = 1.0, float g = 0.0, float b = 0.0);
  void ImageNavigation(void);

  const int interval_ms;
  const double linear_max_;
  const double angular_max_;
  const bool visualize_flag_;
  bool autonomous_flag_=false;

  cv_bridge::CvImagePtr cv_img;
  cv_bridge::CvImagePtr cv_depth_img;

  controller::PositionPid pid;

  imagenav::LineDetector line;
  imagenav::ObstacleDetector obstacle;
};

}  // namespace imagenav
