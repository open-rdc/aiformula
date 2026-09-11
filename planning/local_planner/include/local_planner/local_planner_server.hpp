#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    void global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
    void objects_callback(const object_detection_msgs::msg::ObjectInfoArray::ConstSharedPtr msg);
    void timer_callback();

    // 計画フレーム(odom)へのTFを引く。引けないときはWARNを出してnulloptを返す
    std::optional<geometry_msgs::msg::TransformStamped> lookup_to_odom(
        const std::string& source_frame, const tf2::TimePoint& time);

    pluginlib::ClassLoader<LocalPlannerPlugin> plugin_loader_;
    LocalPlannerPlugin::SharedPtr plugin_;

    const int interval_ms_;
    const rclcpp::QoS qos_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path::ConstSharedPtr global_path_;  // 受信したままの経路(変換前)
    geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr velocity_;
    std::optional<object_detection_msgs::msg::ObjectInfoArray> objects_;  // odom系に変換済み
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<object_detection_msgs::msg::ObjectInfoArray>::SharedPtr objects_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
