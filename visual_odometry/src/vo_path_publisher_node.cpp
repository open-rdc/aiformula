#include "visual_odometry/vo_path_publisher_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <vector>

namespace visual_odometry
{

VoPathPublisher::VoPathPublisher(const rclcpp::NodeOptions & options)
: Node("vo_path_publisher", options)
{
  freq_ = declare_parameter("interval_ms", 100);
  path_file_name_ = declare_parameter("path_file_name", "vo_path");

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
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s", file_path_.c_str());
    return;
  }

  std::string line;
  bool header = true;

  while (std::getline(file, line)) {
    if (header) { header = false; continue; }

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> tokens;

    while (std::getline(ss, cell, ',')) {
      tokens.push_back(cell);
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
  path.header.frame_id = "map";
  path.header.stamp = now();

  for (size_t i = 0; i < xs.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = xs[i];
    pose.pose.position.y = ys[i];
    pose.pose.position.z = 0.0;
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

