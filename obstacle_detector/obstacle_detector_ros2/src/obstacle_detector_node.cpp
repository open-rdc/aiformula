#include "obstacle_detector_ros2/obstacle_detector_node.hpp"

namespace obstacle_detector_ros2
{

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node"),
  image_topic_("/zed/zed_node/rgb/image_rect_color"),
  output_topic_("/object_info")
{
  RCLCPP_INFO(this->get_logger(), "Initializing ObstacleDetectorNode");

  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10,
    std::bind(&ObstacleDetectorNode::image_callback, this, std::placeholders::_1));

  object_info_publisher_ = this->create_publisher<obstacle_detector_msgs::msg::ObjectInfoArray>(
    output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
}

void ObstacleDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    obstacle_detector_msgs::msg::ObjectInfoArray object_info_array;
    object_info_array.header = msg->header;

    object_info_publisher_->publish(object_info_array);

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

}  // namespace obstacle_detector_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obstacle_detector_ros2::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}