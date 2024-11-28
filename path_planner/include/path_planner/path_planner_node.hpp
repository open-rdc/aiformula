#pragma once

#include <rclcpp/rclcpp.hpp>
#include "path_planner/visibility_control.h"

namespace path_planner{

class PathPlannerNode : public rclcpp::Node {
public:
  PATH_PLANNER_PUBLIC
  explicit PathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  PATH_PLANNER_PUBLIC
  explicit PathPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscriptions<sensor_msgs::msg::Image>::SharedPtr line_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  
  sensor_msgs::msg::Image::SharedPtr image_;

  rclcpp::TimerBase::SharedPtr timer_;

  void ImageCallback(const sensor_msgs::msg::Image img) {
    image_ = img;
  }
  
  void PathPlanning();
};

}  // namespace path_planner