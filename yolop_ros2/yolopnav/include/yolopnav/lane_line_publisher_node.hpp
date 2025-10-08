#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
#include "yolopnav/cubic_curve_fitter.hpp"
#include "yolopnav/lane_kalman_filter.hpp"

namespace yolopnav {

class LaneLinePublisher : public rclcpp::Node {
public:
    YOLOPNAV_PUBLIC
    explicit LaneLinePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    YOLOPNAV_PUBLIC
    explicit LaneLinePublisher(const std::string& name_space, 
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_mask_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skeleton_debug_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_raw_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_raw_points_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_ = rclcpp::QoS(10);
    
    std::unique_ptr<LanePixelFinder> lane_pixel_finder_;
    std::unique_ptr<LanePixelToPoint> pixel_to_point_converter_;
    std::unique_ptr<CubicCurveFitter> cubic_curve_fitter_;
    std::unique_ptr<LaneKalmanManager> kalman_manager_;
    
    const int interval_ms_;
    
    sensor_msgs::msg::Image::SharedPtr latest_mask_image_;
    std::mutex image_mutex_;
    
    
    void maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void controlTimerCallback();
    cv::Mat skeletonizeMask(const cv::Mat& mask);
    cv::Mat removeHorizontalLines(const cv::Mat& skeleton_mask, int min_horizontal_length = 30);
    void processLaneDetectionAndControl(const cv::Mat& mask_image);
    void visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness = 2);
    void visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness = 2);
    
    // Point cloud publishing functions
    sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id);
    sensor_msgs::msg::PointCloud2 createPointCloud2FromPixels(const std::vector<cv::Point>& pixels, const std::string& frame_id);

    // RANSAC cubic curve fitting and uniform point extraction
    std::vector<Eigen::Vector3d> fitCurveAndExtractUniformPoints(const std::vector<Eigen::Vector3d>& points);
    
};

}  // namespace yolopnav