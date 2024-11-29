#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "path_planner/visibility_control.h"

namespace path_planner{

class PathPlannerNode : public rclcpp::Node {
public:
  PATH_PLANNER_PUBLIC
  explicit PathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  PATH_PLANNER_PUBLIC
  explicit PathPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr line_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  cv_bridge::CvImagePtr cv_img;
  // sensor_msgs::msg::Image image_;

  rclcpp::TimerBase::SharedPtr timer_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img) {
    cv_img = cv_bridge::toCvCopy(img, img->encoding);
  }
  void PathPlanning();

  const int interval_ms;
  const std::vector<cv::Point2f> srcPoints = {
    cv::Point2f(150, 1080),
    cv::Point2f(760, 540),
    cv::Point2f(1140, 540),
    cv::Point2f(1650, 1080)
  };
  const std::vector<cv::Point2f> dstPoints = {
    cv::Point2f(0, 1080),    
    cv::Point2f(0, 0),
    cv::Point2f(1920, 0),
    cv::Point2f(1920, 1080)
  };
};

}  // namespace path_planner