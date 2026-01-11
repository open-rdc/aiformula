#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <string>

namespace vonav
{

class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const rclcpp::NodeOptions & options);

private:
  void initCommunication();
  void loadCSV();
  void loop();

  nav_msgs::msg::Path createPath(
    const std::vector<double>& xs,
    const std::vector<double>& ys);

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

