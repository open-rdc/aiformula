#include "visual_odometry/vo_path_publisher_node.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

namespace visual_odometry
{

VoPathPublisher::VoPathPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_path_publisher_node", options)
{
  // パラメータ宣言
  this->declare_parameter<std::string>("file_path", "shihou_vo.csv");
  this->declare_parameter<int>("interval_ms", 100);

  std::string file_path =
    this->get_parameter("file_path").get_parameter_value().get<std::string>();
  interval_ms_ =
    this->get_parameter("interval_ms").get_parameter_value().get<int>();

  RCLCPP_INFO(this->get_logger(), "VO path CSV: %s", file_path.c_str());
  RCLCPP_INFO(this->get_logger(), "interval_ms: %d", interval_ms_);

  // CSV 読み込み
  loadCsv(file_path);

  // publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path", 10);

  // timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(interval_ms_),
    std::bind(&VoPathPublisher::timerCallback, this));
}

void VoPathPublisher::loadCsv(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s", file_path.c_str());
    return;
  }

  std::string line;
  bool first_line = true;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> tokens;

    while (std::getline(ss, cell, ',')) {
      tokens.push_back(cell);
    }

    // 先頭行（x,y,z,yaw）はヘッダなのでスキップ
    if (first_line) {
      first_line = false;
      if (!tokens.empty() && tokens[0] == "x") {
        continue;
      }
    }

    if (tokens.size() < 4) {
      continue;
    }

    double x   = std::stod(tokens[0]);
    double y   = std::stod(tokens[1]);
    double z   = std::stod(tokens[2]);
    double yaw = std::stod(tokens[3]);

    if (init_flag_) {
      base_x_ = x;
      base_y_ = y;
      base_z_ = z;
      init_flag_ = false;
    }

    xs_.push_back(x - base_x_);
    ys_.push_back(y - base_y_);
    zs_.push_back(z - base_z_);
    yaws_.push_back(yaw);
  }

  file.close();

  // Path メッセージを作成
  path_msg_.header.frame_id = "map";

  for (size_t i = 0; i < xs_.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = xs_[i];
    pose.pose.position.y = ys_[i];
    pose.pose.position.z = zs_[i];

    // yaw → quaternion は必要なら後で追加
    pose.pose.orientation.w = 1.0;

    path_msg_.poses.push_back(pose);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu points from CSV", xs_.size());
}

void VoPathPublisher::timerCallback()
{
  auto now = this->now();
  path_msg_.header.stamp = now;

  for (auto & pose : path_msg_.poses) {
    pose.header.stamp = now;
  }

  path_pub_->publish(path_msg_);
}

}  // namespace visual_odometry
