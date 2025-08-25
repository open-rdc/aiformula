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
    // 自律走行フラグが無効の場合は制御を行わない
    if (!autonomous_flag_) {
        return;
    }
    
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
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(current_image, sensor_msgs::image_encodings::BGR8);
        processLaneDetectionAndControl(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void LaneLinePublisher::processLaneDetectionAndControl(const cv::Mat& mask_image) {
    // 1. 細線化処理
    cv::Mat skeletonized_mask = skeletonizeMask(mask_image);
    
    // 2. 横線フィルタリング
    cv::Mat filtered_mask = filterHorizontalLines(skeletonized_mask);

    // 3. 白線ピクセル抽出
    LaneLines lane_lines;
    lane_pixel_finder_->searchMask(filtered_mask, lane_lines);
    
    std::vector<cv::Point> left_pixels_to_use = lane_lines.left.pixels;
    std::vector<cv::Point> right_pixels_to_use = lane_lines.right.pixels;
    
    if (lane_lines.left.pixels.size() < MIN_PIXEL_THRESHOLD) {
        if (!prev_left_pixels_.empty()) {
            left_pixels_to_use = prev_left_pixels_;
            RCLCPP_WARN(this->get_logger(), "Left lane pixels insufficient (%zu < %d), using previous result", 
                       lane_lines.left.pixels.size(), MIN_PIXEL_THRESHOLD);
        }
    } else {
        prev_left_pixels_ = lane_lines.left.pixels;
    }
    
    if (lane_lines.right.pixels.size() < MIN_PIXEL_THRESHOLD) {
        if (!prev_right_pixels_.empty()) {
            right_pixels_to_use = prev_right_pixels_;
            RCLCPP_WARN(this->get_logger(), "Right lane pixels insufficient (%zu < %d), using previous result", 
                       lane_lines.right.pixels.size(), MIN_PIXEL_THRESHOLD);
        }
    } else {
        // 十分なpixel数がある場合は前回の結果を更新
        prev_right_pixels_ = lane_lines.right.pixels;
    }
    
    // ピクセル可視化用のデバッグ画像を作成
    cv::Mat debug_image = lane_pixel_finder_->visualizeLanePixels(filtered_mask, lane_lines);
    publishSkeletonDebugImage(debug_image);

    // 4. 線形近似による直線推定（使用するpixelデータで実行）
    std::vector<cv::Vec4i> left_lines = approximateLineFromPixels(left_pixels_to_use, filtered_mask);
    std::vector<cv::Vec4i> right_lines = approximateLineFromPixels(right_pixels_to_use, filtered_mask);
    
    cv::Vec4i best_left_line = left_lines[0];
    double max_left_length = std::sqrt(std::pow(left_lines[0][2] - left_lines[0][0], 2) + std::pow(left_lines[0][3] - left_lines[0][1], 2));
    for (const auto& line : left_lines) {
        double length = std::sqrt(std::pow(line[2] - line[0], 2) + std::pow(line[3] - line[1], 2));
        if (length > max_left_length) {
            max_left_length = length;
            best_left_line = line;
        }
    }
    
    cv::Vec4i best_right_line = right_lines[0];
    double max_right_length = std::sqrt(std::pow(right_lines[0][2] - right_lines[0][0], 2) + std::pow(right_lines[0][3] - right_lines[0][1], 2));
    for (const auto& line : right_lines) {
        double length = std::sqrt(std::pow(line[2] - line[0], 2) + std::pow(line[3] - line[1], 2));
        if (length > max_right_length) {
            max_right_length = length;
            best_right_line = line;
        }
    }
    
    // 5. 交点計算
    cv::Point2f intersection = calculateIntersection(best_left_line, best_right_line);
    
    visualizeLines(debug_image, left_lines, cv::Scalar(0, 255, 255), 1);  // 黄色：左側のピクセル
    visualizeLines(debug_image, right_lines, cv::Scalar(255, 0, 255), 1); // マゼンタ：右側のピクセル

    // 選択された最適な直線と交点を描画
    visualizeLines(debug_image, best_left_line, cv::Scalar(255, 0, 0), 3);   // 青：左側
    visualizeLines(debug_image, best_right_line, cv::Scalar(0, 0, 255), 3);  // 赤：右側
    cv::circle(debug_image, intersection, 10, cv::Scalar(0, 255, 0), -1);
    
    // 6. 制御指令計算
    double angular_velocity = calculateControlCommand(intersection, mask_image.cols);
    
    // 7. 制御指令配信
    publishControlCommand(angular_velocity);
    publishDebugImage(debug_image);
}

std::vector<cv::Vec4i> LaneLinePublisher::approximateLineFromPixels(const std::vector<cv::Point>& pixels, const cv::Mat& image_for_debug) {
    if (pixels.empty()) {
        return {};
    }
    
    // 線形近似を実行
    cv::Vec4f line_params;
    cv::fitLine(pixels, line_params, cv::DIST_L2, 0, 0.01, 0.01);
    
    float vx = line_params[0];
    float vy = line_params[1]; 
    float x0 = line_params[2];
    float y0 = line_params[3];
    
    int img_height = image_for_debug.rows;
    int img_width = image_for_debug.cols;
    
    int x1, y1, x2, y2;
    
    if (std::abs(vy) > std::abs(vx)) {
        y1 = 0;
        x1 = static_cast<int>(x0 - (y0 / vy) * vx);
        y2 = img_height - 1;
        x2 = static_cast<int>(x0 + ((img_height - y0) / vy) * vx);

        x1 = std::max(0, std::min(img_width - 1, x1));
        x2 = std::max(0, std::min(img_width - 1, x2));
    } else {
        x1 = 0;
        y1 = static_cast<int>(y0 - (x0 / vx) * vy);
        x2 = img_width - 1;
        y2 = static_cast<int>(y0 + ((img_width - x0) / vx) * vy);

        y1 = std::max(0, std::min(img_height - 1, y1));
        y2 = std::max(0, std::min(img_height - 1, y2));
    }

    std::vector<cv::Vec4i> lines;
    lines.push_back(cv::Vec4i(x1, y1, x2, y2));

    return lines;
}

cv::Point2f LaneLinePublisher::calculateIntersection(const cv::Vec4i& left_line, const cv::Vec4i& right_line) {
    // 直線の方程式: y = mx + b
    double m1 = static_cast<double>(left_line[3] - left_line[1]) / (left_line[2] - left_line[0]);
    double b1 = left_line[1] - m1 * left_line[0];
    
    double m2 = static_cast<double>(right_line[3] - right_line[1]) / (right_line[2] - right_line[0]);
    double b2 = right_line[1] - m2 * right_line[0];
    
    // 交点計算
    double x = (b2 - b1) / (m1 - m2);
    double y = m1 * x + b1;
    
    return cv::Point2f(x, y);
}

double LaneLinePublisher::calculateControlCommand(const cv::Point2f& intersection_point, int image_width) {

    double center_x = image_width / 2.0;
    double error = center_x - intersection_point.x;
    
    // 正規化（-1.0 to 1.0）
    error = error / (image_width / 2.0);
    
    // PID制御
    double control_output = pid.cycle(error);
    
    control_output = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, control_output));
    
    return control_output;
}

void LaneLinePublisher::publishControlCommand(double angular_velocity) {
    geometry_msgs::msg::Twist cmd_vel;
    
    cmd_vel.linear.x = this->get_parameter("linear_max.vel").as_double();
    cmd_vel.angular.z = angular_velocity;
    
    cmd_vel_publisher_->publish(cmd_vel);
}

void LaneLinePublisher::publishDebugImage(const cv::Mat& debug_image) {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_link";
    
    sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
    debug_image_publisher_->publish(*debug_msg);
}

void LaneLinePublisher::publishSkeletonDebugImage(const cv::Mat& skeleton_image) {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_link";
    
    // グレースケール画像をBGRに変換してより見やすくする
    cv::Mat skeleton_bgr;
    if (skeleton_image.channels() == 1) {
        cv::cvtColor(skeleton_image, skeleton_bgr, cv::COLOR_GRAY2BGR);
    } else {
        skeleton_bgr = skeleton_image.clone();
    }
    
    sensor_msgs::msg::Image::SharedPtr skeleton_msg = cv_bridge::CvImage(header, "bgr8", skeleton_bgr).toImageMsg();
    skeleton_debug_publisher_->publish(*skeleton_msg);
}

cv::Mat LaneLinePublisher::denoiseMask(const cv::Mat& mask) {
    cv::Mat denoised;
    
    cv::GaussianBlur(mask, denoised, cv::Size(3, 3), 0);
    
    // モルフォロジー演算でノイズ除去
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(denoised, denoised, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(denoised, denoised, cv::MORPH_OPEN, kernel);
    
    return denoised;
}

cv::Mat LaneLinePublisher::filterHorizontalLines(const cv::Mat& mask) {
    cv::Mat gray_mask;
    if (mask.channels() == 3) {
        cv::cvtColor(mask, gray_mask, cv::COLOR_BGR2GRAY);
    } else {
        gray_mask = mask.clone();
    }
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(gray_mask, lines, 1, CV_PI/180, 30, 20, 10);
    
    // 横線（角度が水平に近い線）を特定してマスクから除去
    cv::Mat filtered_mask = mask.clone();
    
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        
        double angle = std::atan2(y2 - y1, x2 - x1) * 180.0 / CV_PI;
        angle = std::abs(angle);
        
        if (angle <= 5.0 || angle >= 175.0) {
            cv::line(filtered_mask, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 0), 5);
        }
    }
    
    return filtered_mask;
}

cv::Mat LaneLinePublisher::skeletonizeMask(const cv::Mat& mask) {
    // 1. ノイズ除去
    cv::Mat denoised_mask = denoiseMask(mask);
    
    // 2. 二値化とモルフォロジー処理
    cv::Mat processed_mask;
    if (denoised_mask.channels() == 3) {
        cv::Mat bgr_channels[3];
        cv::split(denoised_mask, bgr_channels);
        cv::Mat red_channel = bgr_channels[2];
        cv::Mat blue_channel = bgr_channels[0];
        cv::Mat red_only;
        cv::subtract(red_channel, blue_channel, red_only);
        cv::threshold(red_only, processed_mask, 100, 255, cv::THRESH_BINARY);
    } else {
        cv::threshold(denoised_mask, processed_mask, 100, 255, cv::THRESH_BINARY);
    }
    
    // 3. モルフォロジー処理
    cv::Mat morphed_mask;
    cv::Mat element_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(processed_mask, morphed_mask, cv::MORPH_CLOSE, element_close);
    cv::Mat element_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
    cv::morphologyEx(morphed_mask, morphed_mask, cv::MORPH_OPEN, element_open);
    
    // 4. 細線化処理
    cv::Mat skeleton = morphed_mask.clone();
    cv::Mat temp;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    
    bool done = false;
    while (!done) {
        cv::morphologyEx(skeleton, temp, cv::MORPH_OPEN, element);
        cv::bitwise_not(temp, temp);
        cv::bitwise_and(skeleton, temp, temp);
        cv::morphologyEx(skeleton, skeleton, cv::MORPH_ERODE, element);
        
        double max_val;
        cv::minMaxLoc(skeleton, nullptr, &max_val);
        done = (max_val == 0);
        
        cv::bitwise_or(skeleton, temp, skeleton);
    }
    
    return skeleton;
}

void LaneLinePublisher::visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness) {
    for (const auto& line : lines) {
        cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, thickness);
    }
}

void LaneLinePublisher::visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness) {
    cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, thickness);
}

}  // namespace yolopnav