#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "imagenav/obstacle_detector.hpp"
#include "imagenav/line_detector.hpp"

#include "imagenav/visibility_control.h"

namespace imagenav{

typedef struct{
  double x;
  double y;
}Position;

class ImageNav : public rclcpp::Node{
public:
  IMAGENAV_PUBLIC
  explicit ImageNav(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  IMAGENAV_PUBLIC
  explicit ImageNav(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;


  rclcpp::TimerBase::SharedPtr timer_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double AttractiveForce(const Position& robot, const Position& goal);
  double RepulsiveForce(const Position& robot, const std::vector<Position>& obstacles);
  double CalculatePotential(Position& robot, double bias_x, double bias_y);

  void PotentialMethod();
  int detected_line_x = 0;

  const double delta=0.1; // 勾配計算パラメータ
  double linear_vel=3.0;

  Position self_pose_;

  imagenav::LineDetector line;
  imagenav::ObstacleDetector obstacle;
};

}  // namespace imagenav
