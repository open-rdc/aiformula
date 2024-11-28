#include "path_planner/path_planner_node.hpp"

namespace path_planner{

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions& options) : PathPlannerNode("", options) {}

PathPlannerNode::PathPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("path_planner_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int())
{
    line_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/line_detections/image", 10, std::bind(&PathPlannerNode::ImageCallback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/upstream_image", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        std::bind(&PathPlannerNode::PathPlanning, this)
    );
}

void PathPlannerNode::PathPlanning(){
    image_pub_->publish(image_);
}

}  // namespace path_planner
