#include "visual_odometry/vo_path_publisher_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace visual_odometry
{

VoPathPublisher::VoPathPublisher(const rclcpp::NodeOptions & options)
: Node("vo_path_publisher", options)
{
  freq_ = declare_parameter("interval_ms", 100);
  path_file_name_ = declare_parameter("path_file_name", "shihou_vo");

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path", 10);

  file_path_ =
    ament_index_cpp::get_package_share_directory("visual_odometry") +
    "/config/course_data/" + path_file_name_ + ".csv";

  loadCSV();
  path_msg_ = createPath(xs_, ys_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(freq_),
    std::bind(&VoPathPublisher::loop, this));

  RCLCPP_INFO(this->get_logger(), "VO Path Publisher initialized");
}

void VoPathPublisher::loadCSV()
{
  std::ifstream file(file_path_);
  std::string line;

  auto trim = [](const std::string & input) {
    const std::string whitespace = " \t\r\n";
    const auto begin = input.find_first_not_of(whitespace);
    if (begin == std::string::npos) return std::string();
    const auto end = input.find_last_not_of(whitespace);
    return input.substr(begin, end - begin + 1);
  };

  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> tokens;

    while (std::getline(ss, cell, ',')) {
      tokens.push_back(trim(cell));
    }

    if (tokens.size() < 2) continue;
    xs_.push_back(std::stod(tokens[0]));
    ys_.push_back(std::stod(tokens[1]));

  }

  RCLCPP_INFO(this->get_logger(),
              "Loaded %zu path points from CSV", xs_.size());
}

nav_msgs::msg::Path
VoPathPublisher::createPath(
  const std::vector<double>& xs,
  const std::vector<double>& ys)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = now();

  for (size_t i = 0; i < xs.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = now();
    pose.pose.position.x = xs[i];
    pose.pose.position.y = ys[i];
    // Force z to 0 and ignore any yaw from source data by using
    // an identity quaternion (no rotation).
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

void VoPathPublisher::loop()
{
  path_msg_.header.stamp = now();
  path_pub_->publish(path_msg_);
}

} 

// ===== main =====
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<visual_odometry::VoPathPublisher>(
            rclcpp::NodeOptions()
        )
    );
    rclcpp::shutdown();
    return 0;
}
