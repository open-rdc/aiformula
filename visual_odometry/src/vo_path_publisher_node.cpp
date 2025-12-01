#include "visual_odmetry/vo_path_publisher_node.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace visual_odometry
{

VoPathPublisher::VoPathPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_path_publisher_node", options)
{
	this->declare_parameter<std::string>("file_path", "shihou_vo.csv");
	this->declare_psrameter<int>("interval_ms", 100);

	std::string file_path =
	  this->get_parameter("file_path").get_parameter_value().get<std::string>();
	interval_ms_ =
	  this->get_parameter("interval_ms").get_parameter_value().get<int>();

	RCLCPP_INFO(this->get_logger(), "VO path CSV: %s", file_path.c_str());
	RCLCPP_INFO(this->get_logger(), "interval_ms: %d", interval_ms_);

	loadCsv(file_path);

	path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path", 10);

	timer_ = this->create_wall_timer(
	  std::chrono::milliseconds(interval_ms_),
	  std::bind(&VoPathPublisher::timerCallback, this));
}

void VoPathPublisher::loadCsv(const std::timerCallback, this));
{
  std::ifstream file(file_path);
  if(!file.is_open()){
   RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s", file_path.c_str()
