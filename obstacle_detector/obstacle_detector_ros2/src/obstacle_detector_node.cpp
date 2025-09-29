#include "obstacle_detector_ros2/obstacle_detector_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace obstacle_detector_ros2
{

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions& options)
: Node("obstacle_detector_node", options),
  image_topic_("/zed/zed_node/rgb/image_rect_color"),
  obstacle_info_topic_("/obstacle_detector/obstacle_info"),
  result_image_topic_("/obstacle_detector/result_image")
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
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10,
    std::bind(&ObstacleDetectorNode::image_callback, this, std::placeholders::_1));

  obstacle_info_publisher_ = this->create_publisher<obstacle_detector_msgs::msg::ObjectInfoArray>(
    obstacle_info_topic_, 10);

  result_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    result_image_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing obstacle_info to: %s", obstacle_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing result_image to: %s", result_image_topic_.c_str());
}

void ObstacleDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Check ROS message validity
    if (msg->width == 0 || msg->height == 0 || msg->width > 10000 || msg->height > 10000) {
      RCLCPP_WARN(this->get_logger(), "Invalid ROS image dimensions: %dx%d. Skipping processing.",
                  msg->width, msg->height);
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Try to convert to BGR8, but handle different input encodings appropriately
      if (msg->encoding == sensor_msgs::image_encodings::BGRA8) {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
        cv::Mat bgra_image = cv_ptr->image;
        cv::Mat bgr_image;
        cv::cvtColor(bgra_image, bgr_image, cv::COLOR_BGRA2BGR);
        cv_ptr->image = bgr_image;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;

    // Check if image is valid
    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cv::Mat image. Skipping processing.");
      return;
    }

    // Additional size sanity check
    if (image.rows <= 0 || image.cols <= 0 || image.rows > 10000 || image.cols > 10000) {
      RCLCPP_WARN(this->get_logger(), "Invalid cv::Mat dimensions: %dx%d. Skipping processing.",
                  image.cols, image.rows);
      return;
    }

    // Detect red objects
    std::vector<cv::Rect> red_bboxes = detect_red_objects(image);

    // Create obstacle info array
    obstacle_detector_msgs::msg::ObjectInfoArray obstacle_info_array;
    obstacle_info_array.header = msg->header;
    obstacle_info_array.header.frame_id = "base_link";

    uint32_t current_object_id = 0;
    for (const auto& bbox : red_bboxes) {
      // Calculate bottom left and right pixels of bbox (following reference implementation)
      cv::Point2f bottom_left(bbox.x, bbox.y + bbox.height);
      cv::Point2f bottom_right(bbox.x + bbox.width, bbox.y + bbox.height);

      // Convert pixels to robot coordinates using ground plane projection
      cv::Point3f bottom_left_point, bottom_right_point;
      bool left_success = pixel_to_robot_point(cv::Point(bottom_left.x, bottom_left.y), bottom_left_point);
      bool right_success = pixel_to_robot_point(cv::Point(bottom_right.x, bottom_right.y), bottom_right_point);

      if (left_success && right_success) {
        obstacle_detector_msgs::msg::ObjectInfo obstacle_info;
        obstacle_info.id = current_object_id++;
        // Calculate center position
        obstacle_info.x = (bottom_left_point.x + bottom_right_point.x) * 0.5;
        obstacle_info.y = (bottom_left_point.y + bottom_right_point.y) * 0.5;
        // Calculate width based on distance between left and right points
        obstacle_info.width = std::abs(bottom_left_point.y - bottom_right_point.y);
        obstacle_info.confidence = red_params_.default_confidence;

        obstacle_info_array.objects.push_back(obstacle_info);
      }
    }

    // Publish obstacle info
    obstacle_info_publisher_->publish(obstacle_info_array);

    // Publish result image with bounding boxes
    publish_result_image(image, red_bboxes, msg->header);

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

std::vector<cv::Rect> ObstacleDetectorNode::detect_red_objects(const cv::Mat& image)
{
  std::vector<cv::Rect> bboxes;

  // Convert BGR to HSV
  cv::Mat hsv;
  try {
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv::cvtColor failed: %s", e.what());
    return bboxes;
  }

  // Create masks for red color in HSV space
  cv::Mat mask1, mask2, red_mask;
  try {
    cv::inRange(hsv, red_params_.lower_red_1, red_params_.upper_red_1, mask1);
    cv::inRange(hsv, red_params_.lower_red_2, red_params_.upper_red_2, mask2);
    red_mask = mask1 | mask2;
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "inRange operations failed: %s", e.what());
    return bboxes;
  }

  // Morphological operations - DISABLED due to crash on Jetson platform
  // Relying on contour area filtering instead

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  try {
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "findContours failed: %s", e.what());
    return bboxes;
  }

  // Filter contours and create bounding boxes
  try {
    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);
      if (area >= red_params_.min_contour_area && area <= red_params_.max_contour_area) {
        cv::Rect bbox = cv::boundingRect(contour);
        bboxes.push_back(bbox);
      }
    }
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Contour filtering failed: %s", e.what());
    return bboxes;
  }

  return bboxes;
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

void ObstacleDetectorNode::publish_result_image(const cv::Mat& image, const std::vector<cv::Rect>& bboxes, const std_msgs::msg::Header& header)
{
  try {
    cv::Mat result_image = image.clone();

    // Draw bounding boxes
    for (size_t i = 0; i < bboxes.size(); ++i) {
      const auto& bbox = bboxes[i];
      cv::rectangle(result_image, bbox, cv::Scalar(0, 255, 0), 2);

      // Add object ID text
      std::string id_text = "ID: " + std::to_string(i);
      cv::putText(result_image, id_text, cv::Point(bbox.x, bbox.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // Convert back to ROS message
    try {
      cv_bridge::CvImage result_cv_image;
      result_cv_image.header = header;
      result_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
      result_cv_image.image = result_image;

      auto image_msg = result_cv_image.toImageMsg();
      result_image_publisher_->publish(*image_msg);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in result image: %s", e.what());
    }
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception in publish_result_image: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Standard exception in publish_result_image: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception in publish_result_image");
  }
}

}  // namespace obstacle_detector_ros2