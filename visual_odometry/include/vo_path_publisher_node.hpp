#pragma once
#include <rclpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <vector>

namespace visual_odometry
{

class VoPathPublisher : public rclcpp::Node
{
public:
  explicit VoPathPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadCsv(const std::string & file_path);
  void timerCallback();

  std::vector<double> xs_;
  std::vector<double> ys_;
  std::vector<double> zs_;
  std::vector<double> yaws_;

  nav_msgs::msg::Path path_msg_;

  bool init_flag_{true};
  double base_x_{0.0};
  double base_y_{0.0};
  double base_z_{0.0};

  int interval_ms_{100};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace visual_odometry
