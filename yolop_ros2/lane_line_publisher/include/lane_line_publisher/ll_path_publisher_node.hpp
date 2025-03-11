#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include "lane_line_publisher/visibility_control.h"

namespace lane_line_publisher {

class LANE_LINE_PUBLISHER_PUBLIC PathPublisherNode : public rclcpp::Node {
    public:
        explicit PathPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    private:
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        std::vector<float> cubic_spline_interpolation(const std::vector<float>& x, 
                                                        const std::vector<float>& y, 
                                                        const std::vector<float>& x_interp);
    
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
        geometry_msgs::msg::Pose current_pose_;
        bool current_pose_received_;
        sensor_msgs::msg::PointCloud2::SharedPtr latest_pc_msg_;
    };
    
}  // namespace lane_line_publisher
