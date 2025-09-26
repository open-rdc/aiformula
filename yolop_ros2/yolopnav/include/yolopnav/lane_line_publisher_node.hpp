#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include "yolopnav/potential_field_generator.hpp"
#include "yolopnav/mpc_controller.hpp"
#include "yolopnav/extremum_seeking_mpc.hpp"
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_ = rclcpp::QoS(10);
    
    // 車線検出
    std::unique_ptr<LanePixelFinder> lane_pixel_finder_;
    
    // 3. 座標変換
    std::unique_ptr<LanePixelToPoint> pixel_to_point_converter_;
    
    // 5. ポテンシャル場による経路生成
    std::unique_ptr<PotentialFieldGenerator> potential_field_generator_;
    
    // 6. MPC制御
    std::unique_ptr<MPCController> mpc_controller_;
    VehicleState current_vehicle_state_;
    
    // 7. Extremum Seeking MPC制御（aiformula準拠）
    std::unique_ptr<ExtremumSeekingMPC> extremum_seeking_mpc_;
    
    // 8. ポテンシャル場可視化
    std::unique_ptr<PotentialFieldVisualizer> potential_field_visualizer_;
    
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
    
    
    // コールバック関数
    void maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void controlTimerCallback();
    void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // 画像処理関数
    cv::Mat skeletonizeMask(const cv::Mat& mask);
    cv::Mat removeHorizontalLines(const cv::Mat& skeleton_mask, int min_horizontal_length = 30);
    
    // 車線検出・制御関数
    void processLaneDetectionAndControl(const cv::Mat& mask_image);
    void publishDebugImage(const cv::Mat& debug_image);
    void visualizeLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color, int thickness = 2);
    void visualizeLines(cv::Mat& image, const cv::Vec4i& line, const cv::Scalar& color, int thickness = 2);
    
    // Point cloud publishing functions
    void publishLanePointClouds(const LaneLines& lane_lines);
    sensor_msgs::msg::PointCloud2 createPointCloud2(const std::vector<Eigen::Vector3d>& points, const std::string& frame_id);
    
    // Potential field path generation
    void generateAndPublishPath(const LaneLines& lane_lines);
    
    // MPC control
    void updateVehicleState(const ControlInput& control_input);
    
    // Extremum Seeking MPC control (aiformula準拠)
    void calculateAndPublishExtremumSeekingControl(const LaneLines& lane_lines);
    
    // Potential field visualization
    void generateAndPublishPotentialFieldVisualization(const LaneLines& lane_lines);
};

}  // namespace yolopnav