#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <speed_path_msgs/msg/speed_path.hpp>
#include <speed_path_msgs/msg/stop_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "speed_path_planner/speed_profile.hpp"
#include "speed_path_planner/visibility_control.h"

namespace speed_path_planner
{

class SpeedPathPlannerNode : public rclcpp::Node
{
public:
    SPEED_PATH_PLANNER_PUBLIC
    explicit SpeedPathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SPEED_PATH_PLANNER_PUBLIC
    explicit SpeedPathPlannerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void stop_point_callback(const speed_path_msgs::msg::StopPoint::ConstSharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
    std::optional<speed_profile::Point2D> lookup_base_position(const std::string& frame_id);
    double start_velocity(
        const std::string& frame_id,
        const std::optional<speed_profile::Point2D>& base_position,
        double v_meas);

    const speed_profile::Limits limits_;
    const double stop_point_timeout_s_;
    const double replan_velocity_deviation_;
    const double previous_plan_timeout_s_;

    // 前回計画（path_callback からのみ触る）
    std::vector<double> previous_s_;
    std::vector<double> previous_velocity_;
    std::string previous_frame_id_;
    speed_profile::Point2D previous_base_position_{0.0, 0.0};
    rclcpp::Time previous_plan_time_{0, 0, RCL_ROS_TIME};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double current_velocity_{0.0};
    double stop_distance_m_{std::numeric_limits<double>::infinity()};
    rclcpp::Time stop_point_time_{0, 0, RCL_ROS_TIME};
    std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<speed_path_msgs::msg::StopPoint>::SharedPtr stop_point_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<speed_path_msgs::msg::SpeedPath>::SharedPtr speed_path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}
