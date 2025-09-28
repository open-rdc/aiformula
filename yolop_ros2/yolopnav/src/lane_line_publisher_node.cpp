#include "yolopnav/lane_line_publisher_node.hpp"
#include <limits>

namespace yolopnav {

LaneLinePublisher::LaneLinePublisher(const rclcpp::NodeOptions& options) 
    : LaneLinePublisher("", options) {}

LaneLinePublisher::LaneLinePublisher(const std::string& name_space, const rclcpp::NodeOptions& options) 
    : Node("yolopnav_node", name_space, options),
      interval_ms_(get_parameter("interval_ms").as_int()) {
    
    lane_pixel_finder_ = std::make_unique<LanePixelFinder>(50);
    pixel_to_point_converter_ = std::make_unique<LanePixelToPoint>();
    cubic_curve_fitter_ = std::make_unique<CubicCurveFitter>(0.1, 100, 4);
    kalman_manager_ = std::make_unique<LaneKalmanManager>();
    
    subscription_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolopv2/image/ll_seg_mask", qos_,
        std::bind(&LaneLinePublisher::maskImageCallback, this, std::placeholders::_1)
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
    
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms_),
        std::bind(&LaneLinePublisher::controlTimerCallback, this)
    );
}

void LaneLinePublisher::maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_mask_image_ = msg;
}


void LaneLinePublisher::controlTimerCallback() {

    sensor_msgs::msg::Image::SharedPtr current_image;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        current_image = latest_mask_image_;
    }
    
    if (!current_image)  return;
    
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
    
    // 2. 横線フィルタリング処理
    cv::Mat filtered_mask = removeHorizontalLines(skelton_mask);

    // 3. 白線ピクセル抽出
    LaneLines lane_lines;
    lane_pixel_finder_->searchMask(filtered_mask, lane_lines);
    
    // 3.5. 白線ピクセル抽出処理の可視化
    cv::Mat debug_image = lane_pixel_finder_->visualizeLanePixels(filtered_mask, lane_lines);
    
    // 4. 座標変換：ピクセル座標をロボット座標系に変換
    lane_lines.left.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.left.pixels); 
    lane_lines.right.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.right.pixels);
    lane_lines.center.points = pixel_to_point_converter_->pixelsToRobotPoints(lane_lines.center.pixels);

    // 5. カルマンフィルタによるフィルタリング
    std::vector<LaneObservation> observations = createLaneObservations(lane_lines);
    kalman_manager_->updateLanes(observations);
    publishKalmanFilteredLanes();
}


cv::Mat LaneLinePublisher::removeHorizontalLines(const cv::Mat& skeleton_mask, int min_horizontal_length) {
    cv::Mat filtered_mask = skeleton_mask.clone();
    
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


std::vector<Eigen::Vector3d> LaneLinePublisher::fitCurveAndExtractUniformPoints(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() < 4) {
        return {};
    }

    // Perform RANSAC-based cubic curve fitting
    FittedCurve fitted_curve = cubic_curve_fitter_->fitCubicCurve(points);

    if (fitted_curve.num_inliers < 4) {
        return {};
    }

    // Fixed x-range for uniform point extraction: 0m to 10m
    const double MIN_X = 0.0;  // Start from x=0[m]
    const double MAX_X = 10.0; // End at x=10[m]
    const double UNIFORM_INTERVAL = 0.5; // 0.5m intervals (magic number)

    std::vector<Eigen::Vector3d> uniform_points = cubic_curve_fitter_->generateUniformPoints(
        fitted_curve, MIN_X, MAX_X, UNIFORM_INTERVAL);

    return uniform_points;
}

std::vector<LaneObservation> LaneLinePublisher::createLaneObservations(const LaneLines& lane_lines) {
    std::vector<LaneObservation> observations;
    auto current_time = std::chrono::steady_clock::now();

    // 左レーンの観測を作成
    if (!lane_lines.left.points.empty()) {
        FittedCurve left_curve = cubic_curve_fitter_->fitCubicCurve(lane_lines.left.points);
        if (left_curve.num_inliers >= 4) {
            LaneObservation obs = createObservationFromCurve(left_curve);
            obs.timestamp = current_time;
            observations.push_back(obs);
        }
    }

    // 右レーンの観測を作成
    if (!lane_lines.right.points.empty()) {
        FittedCurve right_curve = cubic_curve_fitter_->fitCubicCurve(lane_lines.right.points);
        if (right_curve.num_inliers >= 4) {
            LaneObservation obs = createObservationFromCurve(right_curve);
            obs.timestamp = current_time;
            observations.push_back(obs);
        }
    }

    return observations;
}

LaneObservation LaneLinePublisher::createObservationFromCurve(const FittedCurve& curve) {
    LaneObservation observation;

    // y=1m～10mの範囲でサンプリング
    for (double y = 1.0; y <= 10.0; y += 1.0) {
        double x = curve.coefficients.evaluate(y);

        observation.y_positions.push_back(y);
        observation.x_positions.push_back(x);

        // 距離依存の分散を計算
        double base_variance = 0.06; // 基本観測ノイズ [m]
        double distance_factor = 0.02; // 距離係数
        double variance = std::pow(base_variance + distance_factor * y, 2);
        observation.variances.push_back(variance);
    }

    // 信頼度を設定（インライア数とスコアから）
    observation.confidence = std::min(1.0, static_cast<double>(curve.num_inliers) / 20.0);
    if (curve.score > 0) {
        observation.confidence *= std::exp(-curve.score * 10.0); // スコアが小さいほど信頼度高
    }

    return observation;
}

void LaneLinePublisher::publishKalmanFilteredLanes() {
    auto timestamp = this->get_clock()->now();

    // カルマンフィルタの結果を取得して出版
    if (kalman_manager_->hasLeftLane()) {
        auto left_points = kalman_manager_->getLeftLanePoints(0.5);
        if (!left_points.empty()) {
            auto left_cloud = createPointCloud2(left_points, "base_link");
            left_cloud.header.stamp = timestamp;
            left_points_publisher_->publish(left_cloud);
        }
    }

    if (kalman_manager_->hasRightLane()) {
        auto right_points = kalman_manager_->getRightLanePoints(0.5);
        if (!right_points.empty()) {
            auto right_cloud = createPointCloud2(right_points, "base_link");
            right_cloud.header.stamp = timestamp;
            right_points_publisher_->publish(right_cloud);
        }
    }

    auto center_points = kalman_manager_->getCenterLanePoints(0.5);
    if (!center_points.empty()) {
        auto center_cloud = createPointCloud2(center_points, "base_link");
        center_cloud.header.stamp = timestamp;
        center_points_publisher_->publish(center_cloud);
    }

    RCLCPP_DEBUG(this->get_logger(),
                "Published Kalman filtered lanes - Left: %s, Right: %s",
                kalman_manager_->hasLeftLane() ? "Yes" : "No",
                kalman_manager_->hasRightLane() ? "Yes" : "No");
}


}  // namespace yolopnav