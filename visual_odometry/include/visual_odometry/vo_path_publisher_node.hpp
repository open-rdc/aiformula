#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <string>

namespace visual_odometry
{

class VoPathPublisher : public rclcpp::Node
{
public:
  explicit VoPathPublisher(const rclcpp::NodeOptions & options);

private:
  void initCommunication();
  void loadCSV();
  nav_msgs::msg::Path createPath(
    const std::vector<double>& xs,
    const std::vector<double>& ys);
  void loop();

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_msg_;

  std::vector<double> xs_;
  std::vector<double> ys_;

  std::string file_path_;
  std::string path_file_name_;
  int freq_;
};

} // namespace vonav

