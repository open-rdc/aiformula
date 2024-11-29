#include "path_planner/path_planner_node.hpp"

namespace path_planner{

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions& options) : PathPlannerNode("", options) {}

PathPlannerNode::PathPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("path_planner_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int())
{
    line_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/line_detection/image", 10, std::bind(&PathPlannerNode::ImageCallback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/upstream_image", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        std::bind(&PathPlannerNode::PathPlanning, this)
    );
}

void PathPlannerNode::PathPlanning(){
    if(cv_img->image.empty())   return;

    cv::Mat transformMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);
    cv::Mat birdView;

    cv::warpPerspective(cv_img->image, birdView, transformMatrix, cv::Size(1920, 1080));

    // image_pub_->publish(image_);
    sensor_msgs::msg::Image::SharedPtr ros_img = cv_bridge::CvImage(cv_img->header, "bgr8", birdView).toImageMsg();

    image_pub_->publish(*ros_img);
}

}  // namespace path_planner
