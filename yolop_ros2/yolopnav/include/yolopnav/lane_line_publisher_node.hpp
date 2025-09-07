#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <mutex>

#include "yolopnav/visibility_control.h"
#include "yolopnav/lane_pixel_finder.hpp"
#include "yolopnav/lane_pixel_to_point.hpp"
#include "utilities/position_pid.hpp"

namespace yolopnav {

class LaneLinePublisher : public rclcpp::Node {
public:
    YOLOPNAV_PUBLIC
    explicit LaneLinePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    YOLOPNAV_PUBLIC
    explicit LaneLinePublisher(const std::string& name_space, 
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_mask_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skeleton_debug_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_points_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_ = rclcpp::QoS(10);
    
    // 車線検出
    std::unique_ptr<LanePixelFinder> lane_pixel_finder_;
    
    // 3. 座標変換
    std::unique_ptr<LanePixelToPoint> pixel_to_point_converter_;
    
    // 制御周期パラメータ
    const int interval_ms_;
    
    // PIDコントローラ
    controller::PositionPid pid;
    
    // 制御パラメータ
    double max_angular_velocity_; // 最大角速度
    
    // 自律走行フラグ
    bool autonomous_flag_ = false;
    
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
    void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // 画像処理関数
    cv::Mat skeletonizeMask(const cv::Mat& mask);
    cv::Mat interpolateMask(const cv::Mat& skeleton_mask);
    cv::Mat filterHorizontalLines(const cv::Mat& mask);
    cv::Mat denoiseMask(const cv::Mat& mask);
    
    // 車線検出・制御関数
    void processLaneDetectionAndControl(const cv::Mat& mask_image);
    std::vector<cv::Vec4i> approximateLineFromPixels(const std::vector<cv::Point>& pixels, const cv::Mat& image_for_debug);
    cv::Point2f calculateIntersection(const cv::Vec4i& left_line, const cv::Vec4i& right_line);
    double calculateControlCommand(const cv::Point2f& intersection_point, int image_width);
    void publishControlCommand(double angular_velocity);
    void publishDebugImage(const cv::Mat& debug_image);
    void visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness = 2);
    void visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness = 2);
    
    // Point cloud publishing functions
    void publishLanePointClouds(const LaneLines& lane_lines);
    sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id);
};

}  // namespace yolopnav