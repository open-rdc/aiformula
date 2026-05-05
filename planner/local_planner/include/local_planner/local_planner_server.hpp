#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "local_planner/local_planner_plugin.hpp"
#include "local_planner/visibility_control.h"

namespace local_planner
{

class LocalPlannerServer : public rclcpp::Node
{
public:
    LOCAL_PLANNER_PUBLIC
    explicit LocalPlannerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LOCAL_PLANNER_PUBLIC
    explicit LocalPlannerServer(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void objects_callback(const object_detection_msgs::msg::ObjectInfoArray::SharedPtr msg);
    void lane_switch_flag_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void timer_callback();

    pluginlib::ClassLoader<LocalPlannerPlugin> plugin_loader_;
    LocalPlannerPlugin::SharedPtr plugin_;

    const int update_period_ms_;
    const int global_path_timeout_ms_;
    const std::string global_path_topic_;
    const std::string local_path_topic_;
    const std::string vector_map_topic_;
    const std::string localization_pose_topic_;
    const std::string velocity_topic_;
    const std::string objects_topic_;
    const std::string lane_switch_trigger_topic_;
    const rclcpp::QoS qos_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr latest_velocity_;
    object_detection_msgs::msg::ObjectInfoArray::SharedPtr latest_objects_;
    rclcpp::Time last_global_path_stamp_{0, 0, RCL_ROS_TIME};
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<object_detection_msgs::msg::ObjectInfoArray>::SharedPtr objects_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr lane_switch_flag_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace local_planner
