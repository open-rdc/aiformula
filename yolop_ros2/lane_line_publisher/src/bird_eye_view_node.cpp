#include "lane_line_publisher/bird_eye_view_node.hpp"

using std::placeholders::_1;

namespace lane_line_publisher {

BirdEyeViewNode::BirdEyeViewNode(const rclcpp::NodeOptions & options) 
    : Node("bird_eye_view", options) {

    subscription_ll_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolopv2/image/ll_seg_mask", 10, std::bind(&BirdEyeViewNode::ll_callback, this, _1));

    publisher_bev_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/yolopv2/pointcloud2/bird_eye_view", 10);

    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pose", 10);

    src_pts_ = (cv::Mat_<float>(4,2) << 100, 180, 380, 180, -480, 300, 960, 300);
    dst_pts_ = (cv::Mat_<float>(4,2) << 3.0, 2.5, 3.0, -2.5, -1.5, 2.5, -1.5, -2.5);
    M_ = cv::getPerspectiveTransform(src_pts_, dst_pts_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                     std::bind(&BirdEyeViewNode::publish_pose, this));

    RCLCPP_INFO(this->get_logger(), "BirdEyeViewNode initialized.");
}

// 画像処理のコールバック関数
void BirdEyeViewNode::ll_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image data.");

    cv::Mat ll_image;
    try {
        ll_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat red_channel, binary_mask;
    cv::extractChannel(ll_image, red_channel, 2);
    cv::threshold(red_channel, binary_mask, 150, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> points;
    cv::findNonZero(binary_mask, points);
    if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No red regions detected. Skipping processing.");
        return;
    }

    std::vector<cv::Point2f> pixel_coords, transformed_coords;
    for (const auto& p : points) {
        pixel_coords.emplace_back(p.x, p.y);
    }
    cv::perspectiveTransform(pixel_coords, transformed_coords, M_);

    std::map<int, std::pair<cv::Point2f, cv::Point2f>> transformed_dict;
    for (const auto& p : transformed_coords) {
        int x = static_cast<int>(std::round(p.x));
        float y = p.y;

        // RCLCPP_INFO(this->get_logger(), "Transformed points: x=%f, y=%f", p.x, p.y);

        if (transformed_dict.find(x) == transformed_dict.end()) {
            transformed_dict[x] = {{0, 0}, {0, 0}};
        }

        if (y >= 0) {
            if (transformed_dict[x].first == cv::Point2f(0, 0) || std::abs(y) < std::abs(transformed_dict[x].first.y)) {
                transformed_dict[x].first = {static_cast<float>(x), y};
            }
        } else {
            if (transformed_dict[x].second == cv::Point2f(0, 0) || std::abs(y) < std::abs(transformed_dict[x].second.y)) {
                transformed_dict[x].second = {static_cast<float>(x), y};
            }
        }
    }

    std::vector<geometry_msgs::msg::Point32> filtered_points{};
    for (const auto& kv : transformed_dict) {
        if (kv.second.first != cv::Point2f(0, 0)) {
            geometry_msgs::msg::Point32 p;
            p.x = kv.second.first.x;
            p.y = kv.second.first.y;
            p.z = 0.0;
            filtered_points.emplace_back(p);
        }
        if (kv.second.second != cv::Point2f(0, 0)) {
            geometry_msgs::msg::Point32 p;
            p.x = kv.second.second.x;
            p.y = kv.second.second.y;
            p.z = 0.0;
            filtered_points.emplace_back(p);
        }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1;
    cloud_msg.width = filtered_points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(filtered_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& point : filtered_points) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    publisher_bev_->publish(cloud_msg);
    // RCLCPP_INFO(this->get_logger(), "Published filtered PointCloud2 with %ld points.", filtered_points.size());
}

void BirdEyeViewNode::publish_pose() {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = robot_x_;
    pose_msg.pose.position.y = robot_y_;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;

    publisher_pose_->publish(pose_msg);
    // RCLCPP_INFO(this->get_logger(), "Published Pose: x=%.2f, y=%.2f", robot_x_, robot_y_);
}

} // namespace lane_line_publisher