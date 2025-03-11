#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "lane_line_publisher/visibility_control.h"

namespace lane_line_publisher {

class LANE_LINE_PUBLISHER_PUBLIC BirdEyeViewNode : public rclcpp::Node {
public:
    explicit BirdEyeViewNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_ll_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_bev_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat M_;
    cv::Mat src_pts_;
    cv::Mat dst_pts_;

    float robot_x_ = 0.0;
    float robot_y_ = 0.0;
    float robot_yaw_ = 0.0;

    void ll_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publish_pose();
};

} // namespace lane_line_publisher