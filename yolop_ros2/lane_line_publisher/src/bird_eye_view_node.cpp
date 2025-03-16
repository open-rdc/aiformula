#include "lane_line_publisher/bird_eye_view_node.hpp"

namespace lane_line_publisher {

BirdEyeViewNode::BirdEyeViewNode(const rclcpp::NodeOptions & options)
    : Node("bird_eye_view", options),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0)
{
    // サブスクリプションの設定
    subscription_ll_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolopv2/image/ll_seg_mask", 10, std::bind(&BirdEyeViewNode::ll_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    publisher_bev_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/yolopv2/pointcloud2/bird_eye_view", 10);

    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pose", 10);

    // 透視変換行列の設定
    src_pts_ = (cv::Mat_<float>(4,2) << 100, 180, 380, 180, -480, 300, 960, 300);
    dst_pts_ = (cv::Mat_<float>(4,2) << 3.0, 2.5, 3.0, -2.5, 0.0, 2.5, 0.0, -2.5);
    M_ = cv::getPerspectiveTransform(src_pts_, dst_pts_);

    // タイマーの設定
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                     std::bind(&BirdEyeViewNode::publish_pose, this));

    RCLCPP_INFO(this->get_logger(), "BirdEyeViewNode initialized.");
}

// 画像処理のコールバック関数
void BirdEyeViewNode::ll_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image data.");

    cv::Mat ll_image;
    try {
        // 画像メッセージをOpenCVのMatに変換
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        ll_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat red_channel, binary_mask;
    // 赤チャンネルを抽出
    cv::extractChannel(ll_image, red_channel, 2);
    // 二値化処理
    cv::threshold(red_channel, binary_mask, 150, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> points;
    // 非ゼロのピクセルを検出
    cv::findNonZero(binary_mask, points);
    if (points.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No red regions detected. Skipping processing.");
        return;
    }

    std::vector<cv::Point2f> pixel_coords(points.size());
    std::transform(points.begin(), points.end(), pixel_coords.begin(), [](const cv::Point& p) {
        return cv::Point2f(p.x, p.y);
    });

    // 透視変換を適用
    std::vector<cv::Point2f> transformed_coords;
    cv::perspectiveTransform(pixel_coords, transformed_coords, M_);

    std::map<int, std::pair<cv::Point2f, cv::Point2f>> transformed_dict;
    for (const cv::Point2f& p : transformed_coords) {
        int x = static_cast<int>(std::round(p.x));
        float y = p.y;

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

    std::vector<geometry_msgs::msg::Point32> filtered_points;
    float y_limit_pos = 2.0f; // 正のy値の制限
    float y_limit_neg = 1.0f; // 負のy値の制限
    for (const auto& kv : transformed_dict) {
        if (kv.second.first != cv::Point2f(0, 0)) {
            geometry_msgs::msg::Point32 point;
            point.x = kv.second.first.x;
            point.y = std::min(std::max(kv.second.first.y, -y_limit_neg), y_limit_pos); // y値を制限
            point.z = 0.0f;
            filtered_points.push_back(point);
        }
        if (kv.second.second != cv::Point2f(0, 0)) {
            geometry_msgs::msg::Point32 point;
            point.x = kv.second.second.x;
            point.y = std::min(std::max(kv.second.second.y, -y_limit_neg), y_limit_pos); // y値を制限
            point.z = 0.0f;
            filtered_points.push_back(point);
        }
    }

    // x値を1, 2, 3, 4, 5のポイントに制限
    std::vector<int> target_x_vals = {1, 2, 3, 4, 5};
    std::vector<geometry_msgs::msg::Point32> constrained_points;
    for (const auto& point : filtered_points) {
        int x = static_cast<int>(std::round(point.x));
        if (std::find(target_x_vals.begin(), target_x_vals.end(), x) != target_x_vals.end()) {
            constrained_points.push_back(point);
        }
    }

    // PointCloud2メッセージの作成
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1;
    cloud_msg.width = constrained_points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(constrained_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const geometry_msgs::msg::Point32& point : constrained_points) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    // PointCloud2メッセージをパブリッシュ
    publisher_bev_->publish(cloud_msg);
    // RCLCPP_INFO(this->get_logger(), "Published filtered PointCloud2 with %ld points.", constrained_points.size());
}

// ロボットの位置情報をパブリッシュする関数
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