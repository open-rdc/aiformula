#pragma once

#include "path_smoother/smoother.hpp"
#include "path_smoother/visibility_control.h"

#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/path.hpp>

#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace path_smoother {

class PathSmootherNode : public rclcpp::Node {
   public:
    PATH_SMOOTHER_PUBLIC
    explicit PathSmootherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    PATH_SMOOTHER_PUBLIC
    explicit PathSmootherNode(
        const std::string&         name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);

    const double window_duration_s_;
    const double tf_timeout_s_;
    const double fit_max_x_m_;
    const double extend_to_x_m_;
    const double output_interval_m_;
    const double ransac_threshold_m_;
    const int    ransac_iterations_;

    tf2_ros::Buffer                                  tf_buffer_;
    tf2_ros::TransformListener                       tf_listener_;
    message_filters::Subscriber<nav_msgs::msg::Path> path_subscriber_;
    tf2_ros::MessageFilter<nav_msgs::msg::Path>      tf_filter_;
    std::deque<std::pair<rclcpp::Time, std::vector<Point2D>>> window_;  // (画像時刻, odom 座標の点列)。古い順

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

}  // namespace path_smoother
