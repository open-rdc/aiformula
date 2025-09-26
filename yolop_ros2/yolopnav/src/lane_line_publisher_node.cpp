#include "yolopnav/lane_line_publisher_node.hpp"

namespace yolopnav {

LaneLinePublisher::LaneLinePublisher(const rclcpp::NodeOptions& options) 
    : LaneLinePublisher("", options) {}

LaneLinePublisher::LaneLinePublisher(const std::string& name_space, const rclcpp::NodeOptions& options) 
    : Node("yolopnav_node", name_space, options),
      interval_ms_(get_parameter("interval_ms").as_int()),
      pid(get_parameter("interval_ms").as_int()),
      max_angular_velocity_(get_parameter("angular_max.vel").as_double() * M_PI / 180.0) {
    
    lane_pixel_finder_ = std::make_unique<LanePixelFinder>(50);
    pixel_to_point_converter_ = std::make_unique<LanePixelToPoint>();
    potential_field_generator_ = std::make_unique<PotentialFieldGenerator>();
    mpc_controller_ = std::make_unique<MPCController>();
    extremum_seeking_mpc_ = std::make_unique<ExtremumSeekingMPC>();
    potential_field_visualizer_ = std::make_unique<PotentialFieldVisualizer>();
    
    // 車両状態を初期化
    current_vehicle_state_ = VehicleState{0.0, 0.0, 0.0, 0.0, 0.0};
    
    subscription_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolopv2/image/ll_seg_mask", qos_,
        std::bind(&LaneLinePublisher::maskImageCallback, this, std::placeholders::_1)
    );
    
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous", 10,
        std::bind(&LaneLinePublisher::autonomousFlagCallback, this, std::placeholders::_1)
    );
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos_
    );
    
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/debug_image", qos_
    );
    
    skeleton_debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/skeleton_debug_image", qos_
    );
    
    left_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lane_points/left", qos_
    );
    
    right_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lane_points/right", qos_
    );
    
    center_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lane_points/center", qos_
    );
    
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "/potential_field_path", qos_
    );
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms_),
        std::bind(&LaneLinePublisher::controlTimerCallback, this)
    );
    
    pid.gain(get_parameter("pid.kp").as_double(), get_parameter("pid.ki").as_double(), get_parameter("pid.kd").as_double());
}

void LaneLinePublisher::maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_mask_image_ = msg;
}

void LaneLinePublisher::autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    autonomous_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
}

void LaneLinePublisher::controlTimerCallback() {
    if (!autonomous_flag_)  return;
    
    sensor_msgs::msg::Image::SharedPtr current_image;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        current_image = latest_mask_image_;
    }
    
    if (!current_image) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No mask image received yet");
        return;
    }
    
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image, sensor_msgs::image_encodings::MONO8);
        processLaneDetectionAndControl(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LaneLinePublisher::processLaneDetectionAndControl(const cv::Mat& mask_image) {
    // 1. 細線化処理
    cv::Mat skelton_mask = skeletonizeMask(mask_image);
    
    // 2. 横線フィルタリング処理（停止線除去）
    cv::Mat filtered_mask = removeHorizontalLines(skelton_mask);

    // 3. 白線ピクセル抽出
    LaneLines lane_lines;
    lane_pixel_finder_->searchMask(filtered_mask, lane_lines);
    
    // 3.5. 白線ピクセル抽出処理の可視化
    cv::Mat debug_image = lane_pixel_finder_->visualizeLanePixels(filtered_mask, lane_lines);
    publishDebugImage(debug_image);
    
    // 4. 座標変換：ピクセル座標をロボット座標系に変換
    lane_lines.left.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.left.pixels); 
    lane_lines.right.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.right.pixels);
    lane_lines.center.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.center.pixels);        

    // 4.5 座標変換後点群のpublish
    publishLanePointClouds(lane_lines);

    // // 5. 座標変換後の点群を基にポテンシャル場による経路生成
    // generateAndPublishPath(lane_lines);
    
    // // 6. ポテンシャル場の可視化
    // generateAndPublishPotentialFieldVisualization(lane_lines);
    
    // // 7. Extremum Seeking MPCによる制御指令値計算とpublish
    // calculateAndPublishExtremumSeekingControl(lane_lines);

}


void LaneLinePublisher::publishDebugImage(const cv::Mat& debug_image) {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_link";
    
    sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
    debug_image_publisher_->publish(*debug_msg);
}


cv::Mat LaneLinePublisher::removeHorizontalLines(const cv::Mat& skeleton_mask, int min_horizontal_length) {
    cv::Mat filtered_mask = skeleton_mask.clone();
    
    // 各行について横方向の連続白ピクセル長を計算
    for (int y = 0; y < skeleton_mask.rows; y++) {
        const uchar* row_ptr = skeleton_mask.ptr<uchar>(y);
        uchar* filtered_row_ptr = filtered_mask.ptr<uchar>(y);
        
        int consecutive_count = 0;
        int start_x = -1;
        
        for (int x = 0; x <= skeleton_mask.cols; x++) {
            bool is_white = (x < skeleton_mask.cols && row_ptr[x] > 0);
            
            if (is_white) {
                if (consecutive_count == 0) {
                    start_x = x;
                }
                consecutive_count++;
            } else {
                if (consecutive_count >= min_horizontal_length) {
                    for (int remove_x = start_x; remove_x < start_x + consecutive_count; remove_x++) {
                        filtered_row_ptr[remove_x] = 0;
                    }
                }
                consecutive_count = 0;
            }
        }
    }
    
    return filtered_mask;
}

cv::Mat LaneLinePublisher::skeletonizeMask(const cv::Mat& mask) {
    cv::Mat skeleton_mask;
    cv::ximgproc::thinning(mask, skeleton_mask, cv::ximgproc::THINNING_ZHANGSUEN);
    return skeleton_mask;
}

void LaneLinePublisher::visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness) {
    for (const auto& line : lines) {
        cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, thickness);
    }
}

void LaneLinePublisher::visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness) {
    cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, thickness);
}

void LaneLinePublisher::publishLanePointClouds(const LaneLines& lane_lines) {
    auto timestamp = this->get_clock()->now();
    
    if (!lane_lines.left.points.empty()) {
        auto left_cloud = createPointCloud2(lane_lines.left.points, "base_link");
        left_cloud.header.stamp = timestamp;
        left_points_publisher_->publish(left_cloud);
    }
    
    if (!lane_lines.right.points.empty()) {
        auto right_cloud = createPointCloud2(lane_lines.right.points, "base_link");
        right_cloud.header.stamp = timestamp;
        right_points_publisher_->publish(right_cloud);
    }
    
    if (!lane_lines.center.points.empty()) {
        auto center_cloud = createPointCloud2(lane_lines.center.points, "base_link");
        center_cloud.header.stamp = timestamp;
        center_points_publisher_->publish(center_cloud);
    }
    
    RCLCPP_DEBUG(this->get_logger(),
                "Published point clouds - Left: %zu points, Right: %zu points, Center: %zu points",
                lane_lines.left.points.size(), lane_lines.right.points.size(), lane_lines.center.points.size());
}

sensor_msgs::msg::PointCloud2 LaneLinePublisher::createPointCloud2(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    // Set header
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 12;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    
    // Set field descriptions
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    
    // Fill data
    cloud_msg.data.resize(cloud_msg.row_step);
    float* data_ptr = reinterpret_cast<float*>(cloud_msg.data.data());
    
    for (size_t i = 0; i < points.size(); ++i) {
        data_ptr[i * 3 + 0] = static_cast<float>(points[i].x());
        data_ptr[i * 3 + 1] = static_cast<float>(points[i].y());
        data_ptr[i * 3 + 2] = static_cast<float>(points[i].z());
    }
    
    return cloud_msg;
}

void LaneLinePublisher::generateAndPublishPath(const LaneLines& lane_lines) {
    LaneData lane_data;
    lane_data.left_points = lane_lines.left.points;
    lane_data.right_points = lane_lines.right.points;
    lane_data.center_points = lane_lines.center.points;
    
    // ポテンシャル場による経路生成（ロボット座標系では原点から開始）
    nav_msgs::msg::Path path = potential_field_generator_->generatePath(lane_data);
    
    // タイムスタンプを設定
    path.header.stamp = this->get_clock()->now();
    // 経路をパブリッシュ
    path_publisher_->publish(path);
    
    RCLCPP_DEBUG(this->get_logger(), "Generated and published potential field path with %zu points", path.poses.size());
}


void LaneLinePublisher::updateVehicleState(const ControlInput& control_input) {
    double dt = interval_ms_ / 1000.0;
    
    current_vehicle_state_.x += current_vehicle_state_.velocity * std::cos(current_vehicle_state_.yaw) * dt;
    current_vehicle_state_.y += current_vehicle_state_.velocity * std::sin(current_vehicle_state_.yaw) * dt;
    current_vehicle_state_.yaw += current_vehicle_state_.yaw_rate * dt;
    current_vehicle_state_.velocity = control_input.linear_velocity;
    current_vehicle_state_.yaw_rate = control_input.angular_velocity;
}

void LaneLinePublisher::calculateAndPublishExtremumSeekingControl(const LaneLines& lane_lines) {
    LaneData lane_data;
    lane_data.left_points = lane_lines.left.points;
    lane_data.right_points = lane_lines.right.points;
    lane_data.center_points = lane_lines.center.points;
    
    // Extremum Seeking MPCによる最適制御計算
    geometry_msgs::msg::Twist optimal_control = extremum_seeking_mpc_->calculateOptimalControl(lane_data);
    
    // 制御指令をパブリッシュ
    cmd_vel_publisher_->publish(optimal_control);
    
    RCLCPP_DEBUG(this->get_logger(), "Published Extremum Seeking MPC control: linear_vel=%.3f, angular_vel=%.3f", 
                optimal_control.linear.x, optimal_control.angular.z);
}

void LaneLinePublisher::generateAndPublishPotentialFieldVisualization(const LaneLines& lane_lines) {
    // 座標変換後の点群からLaneDataを構築
    LaneData lane_data;
    lane_data.left_points = lane_lines.left.points;
    lane_data.right_points = lane_lines.right.points;
    lane_data.center_points = lane_lines.center.points;
    
    // OpenCVのimshowでリアルタイム表示
    potential_field_visualizer_->showPotentialFieldRealtime(lane_data, *potential_field_generator_);
    
    RCLCPP_DEBUG(this->get_logger(), "Displayed potential field visualization with imshow");
}

}  // namespace yolopnav