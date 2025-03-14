#include "lane_line_publisher/ll_path_publisher_node.hpp"

namespace lane_line_publisher {

PathPublisherNode::PathPublisherNode(const rclcpp::NodeOptions & options)
    : Node("ll_path_publisher_node", options), current_pose_received_(false), latest_pc_msg_(nullptr) {
    // サブスクライバの作成
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/yolopv2/pointcloud2/bird_eye_view", 10,
        std::bind(&PathPublisherNode::pc_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose", 10,
        std::bind(&PathPublisherNode::pose_callback, this, std::placeholders::_1));

    // パブリッシャーの作成
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    RCLCPP_INFO(this->get_logger(), "PathPublisherNode initialized.");
}

void PathPublisherNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // ロボットの現在位置を保存
    current_pose_ = msg->pose;
    current_pose_received_ = true;

    if (latest_pc_msg_) {
        process_pointcloud(latest_pc_msg_);
        latest_pc_msg_ = nullptr;
    }
}

void PathPublisherNode::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!current_pose_received_) {
        RCLCPP_WARN(this->get_logger(), "Pose not received yet. Storing latest PointCloud2 message.");
        latest_pc_msg_ = msg;
        return;
    }
    process_pointcloud(msg);
}

void PathPublisherNode::process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!current_pose_received_) {
        RCLCPP_WARN(this->get_logger(), "Pose not available. Skipping path generation.");
        return;
    }

    // PointCloud2 から (x, y) 座標を取得
    std::vector<std::pair<int, float>> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        float x = *iter_x;
        float y = *iter_y;
        points.emplace_back(static_cast<int>(x), y);
    }

    if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points received in PointCloud2.");
        return;
    }

    // 各 x ごとに y のリストを作成
    std::map<int, std::vector<float>> x_to_y;
    for (const auto& point : points) {
        x_to_y[point.first].push_back(point.second);
    }

    // 各 x ごとに y の中心を計算
    std::vector<std::pair<int, float>> waypoints;
    for (const std::pair<const int, std::vector<float>>& kv : x_to_y) {
        int x = kv.first;
        std::vector<float> y_vals = kv.second;

        if (y_vals.size() >= 2) {
            std::sort(y_vals.begin(), y_vals.end());
            float y_center = (y_vals.front() + y_vals.back()) / 2.0;
            waypoints.emplace_back(x, y_center);
        }
    }

    if (waypoints.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Insufficient path points for interpolation. Skipping path update.");
        return;
    }

    // Path メッセージ作成
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path_msg.header;
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(start_pose);

    // waypoints を基にパスを生成
    for (const std::pair<int, float>& waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = waypoint.first;
        pose.pose.position.y = waypoint.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    // RCLCPP_INFO(this->get_logger(), "Published Path with %zu points.", path_msg.poses.size());
}

std::vector<float> PathPublisherNode::cubic_spline_interpolation(const std::vector<float>& x, 
    const std::vector<float>& y, 
    const std::vector<float>& x_interp) {
std::vector<float> y_interp(x_interp.size());
    for (size_t i = 0; i < x_interp.size(); ++i) {
    float xi = x_interp[i];
        for (size_t j = 0; j < x.size() - 1; ++j) {
            if (xi >= x[j] && xi <= x[j+1]) {
            y_interp[i] = linear_interpolation(x[j], y[j], x[j+1], y[j+1], xi);
            break;
            }
        }
    }
return y_interp;
}

float PathPublisherNode::linear_interpolation(float x0, float y0, float x1, float y1, float x) {
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

}  // namespace lane_line_publisher