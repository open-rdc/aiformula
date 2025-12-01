#include "visual_odmetry/vo_path_publisher_node.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace visual_odometry
{

VoPathPublisher::VoPathPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_path_publisher_node", options)
{
	this
