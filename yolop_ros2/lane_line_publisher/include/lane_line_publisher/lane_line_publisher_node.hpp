#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <mutex>

#include "lane_line_publisher/visibility_control.h"
#include "lane_line_publisher/lane_pixel_finder.hpp"
#include "utilities/position_pid.hpp"

namespace lane_line_publisher {

class LaneLinePublisher : public rclcpp::Node {
public:
    LANE_LINE_PUBLISHER_PUBLIC
    explicit LaneLinePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LANE_LINE_PUBLISHER_PUBLIC
    explicit LaneLinePublisher(const std::string& name_space, 
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_mask_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skeleton_debug_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_ = rclcpp::QoS(10);
    
    // 車線検出
    std::unique_ptr<LanePixelFinder> lane_pixel_finder_;
    
    // 制御周期パラメータ
    const int interval_ms_;
    
    // PIDコントローラ
    controller::PositionPid pid;
    
    // 制御パラメータ
    double max_angular_velocity_; // 最大角速度
    
    // 最新の画像データ
    sensor_msgs::msg::Image::SharedPtr latest_mask_image_;
    std::mutex image_mutex_;
    
    // 前回の検出結果を保存（pixel数が不足した場合の代替用）
    std::vector<cv::Point> prev_left_pixels_;
    std::vector<cv::Point> prev_right_pixels_;
    static constexpr int MIN_PIXEL_THRESHOLD = 30;
    
    // コールバック関数
    void maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void controlTimerCallback();
    
    // 画像処理関数
    cv::Mat skeletonizeMask(const cv::Mat& mask);
    cv::Mat filterHorizontalLines(const cv::Mat& mask);
    cv::Mat denoiseMask(const cv::Mat& mask);
    
    // 車線検出・制御関数
    void processLaneDetectionAndControl(const cv::Mat& mask_image);
    std::vector<cv::Vec4i> approximateLineFromPixels(const std::vector<cv::Point>& pixels, const cv::Mat& image_for_debug);
    cv::Point2f calculateIntersection(const cv::Vec4i& left_line, const cv::Vec4i& right_line);
    double calculateControlCommand(const cv::Point2f& intersection_point, int image_width);
    void publishControlCommand(double angular_velocity);
    void publishDebugImage(const cv::Mat& debug_image);
    void publishSkeletonDebugImage(const cv::Mat& skeleton_image);
    void visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness = 2);
    void visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness = 2);
};

}  // namespace lane_line_publisher