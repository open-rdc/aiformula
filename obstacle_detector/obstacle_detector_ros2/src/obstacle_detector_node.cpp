#include "obstacle_detector_ros2/obstacle_detector_node.hpp"

namespace obstacle_detector_ros2
{

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions& options)
: Node("obstacle_detector_node", options),
  obstacle_pixel_topic_("/obstacle_pixel"),
  obstacle_info_topic_("/obstacle_detector/obstacle_info"),
  obstacle_pos_topic_("/obstacle_detector/obstacle_pos")
{
  RCLCPP_INFO(this->get_logger(), "Initializing ObstacleDetectorNode");

  // Initialize camera parameters
  camera_matrix_ = cv::Mat::eye(3, 3, CV_32F);
  camera_matrix_.at<float>(0, 0) = camera_params_.focal_length_x;
  camera_matrix_.at<float>(1, 1) = camera_params_.focal_length_y;
  camera_matrix_.at<float>(0, 2) = camera_params_.center_point_x;
  camera_matrix_.at<float>(1, 2) = camera_params_.center_point_y;
  invert_camera_matrix_ = camera_matrix_.inv();

  // Create publishers and subscribers
  rect_array_subscription_ = this->create_subscription<obstacle_detector_msgs::msg::RectArray>(
    obstacle_pixel_topic_, 10,
    std::bind(&ObstacleDetectorNode::rect_array_callback, this, std::placeholders::_1));

  obstacle_info_publisher_ = this->create_publisher<obstacle_detector_msgs::msg::ObjectInfoArray>(
    obstacle_info_topic_, 10);

  obstacle_pos_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    obstacle_pos_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", obstacle_pixel_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing obstacle_info to: %s", obstacle_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing obstacle_pos to: %s", obstacle_pos_topic_.c_str());
}

void ObstacleDetectorNode::rect_array_callback(const obstacle_detector_msgs::msg::RectArray::SharedPtr msg)
{
  try {
    // Create obstacle info array
    obstacle_detector_msgs::msg::ObjectInfoArray obstacle_info_array;
    obstacle_info_array.header = msg->header;
    obstacle_info_array.header.frame_id = "base_link";

    // Process each detected rectangle
    for (size_t i = 0; i < msg->rects.size(); ++i) {
      const auto& rect = msg->rects[i];

      // Calculate bottom left and right pixels of bbox
      // rect.x and rect.y represent the center bottom of the bbox
      // rect.width and rect.height are the bbox dimensions
      cv::Point2f bottom_center(rect.x, rect.y);
      cv::Point2f bottom_left(rect.x - rect.width / 2.0, rect.y);
      cv::Point2f bottom_right(rect.x + rect.width / 2.0, rect.y);

      // Convert pixels to robot coordinates using ground plane projection
      cv::Point3f bottom_left_point, bottom_right_point;
      bool left_success = pixel_to_robot_point(cv::Point(bottom_left.x, bottom_left.y), bottom_left_point);
      bool right_success = pixel_to_robot_point(cv::Point(bottom_right.x, bottom_right.y), bottom_right_point);

      if (left_success && right_success) {
        obstacle_detector_msgs::msg::ObjectInfo obstacle_info;
        obstacle_info.id = static_cast<uint32_t>(i);
        // Calculate center position
        obstacle_info.x = (bottom_left_point.x + bottom_right_point.x) * 0.5;
        obstacle_info.y = (bottom_left_point.y + bottom_right_point.y) * 0.5;
        // Calculate width based on distance between left and right points
        obstacle_info.width = std::abs(bottom_left_point.y - bottom_right_point.y);
        obstacle_info.confidence = 0.9;  // Default confidence

        obstacle_info_array.objects.push_back(obstacle_info);
      }
    }

    // Publish obstacle info
    obstacle_info_publisher_->publish(obstacle_info_array);

    // Create and publish pose array
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;
    pose_array.header.frame_id = "base_link";

    // Add poses for each obstacle
    for (const auto& obj : obstacle_info_array.objects) {
      geometry_msgs::msg::Pose pose;

      // Set position
      pose.position.x = obj.x;
      pose.position.y = obj.y;
      pose.position.z = 0.0;

      // Set orientation (identity quaternion)
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;

      pose_array.poses.push_back(pose);
    }

    obstacle_pos_publisher_->publish(pose_array);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in rect_array_callback: %s", e.what());
  }
}


bool ObstacleDetectorNode::pixel_to_robot_point(const cv::Point& pixel, cv::Point3f& robot_point)
{
  // Transform pixel to camera vector using inverse camera matrix (following yolopnav approach)
  cv::Mat pixel_matrix = (cv::Mat_<float>(3, 1) << pixel.x, pixel.y, 1.0);
  cv::Mat camera_vec_matrix = invert_camera_matrix_ * pixel_matrix;

  double x_cam = camera_vec_matrix.at<float>(0, 0);
  double y_cam = camera_vec_matrix.at<float>(1, 0);
  double z_cam = camera_vec_matrix.at<float>(2, 0);

  // For horizontal camera: ground plane projection using y_cam
  // Camera height above ground, bottom of image shows ground (y_cam > 0)
  double scale = camera_params_.camera_height / y_cam;

  // Skip points above horizon (y_cam <= 0) or invalid scale
  if (y_cam <= 0.0 || scale <= 0.0) {
    return false;
  }

  double x_ground = x_cam * scale;
  double z_ground = z_cam * scale;

  // Transform from camera coordinate to robot coordinate
  double x_robot = z_ground;
  double y_robot = -x_ground;
  double z_robot = 0.0;

  robot_point.x = x_robot;
  robot_point.y = y_robot;
  robot_point.z = z_robot;

  return true;
}


}  // namespace obstacle_detector_ros2