#pragma once

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include "path_tracker/visibility_control.h"

namespace path_tracker {

class PurePursuitGlobal : public rclcpp::Node {
public:
    PATH_TRACKER_PUBLIC
    explicit PurePursuitGlobal(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    PATH_TRACKER_PUBLIC
    explicit PurePursuitGlobal(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path_;
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_velocity_;
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_autonomous_;
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_self_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target_pose_;

    void timer_callback();
    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat);

    rclcpp::QoS qos_ = rclcpp::QoS(10);
    rclcpp::TimerBase::SharedPtr timer_;

    bool autonomous_flag_ = false;
    bool has_velocity_ = false;
    bool has_path_ = false;
    bool has_imu_ = false;
    bool has_yaw_reference_ = false;
    double yaw_reference_ = 0.0;
    geometry_msgs::msg::Pose current_pose_ = []{
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        return pose;
    }();
    geometry_msgs::msg::Twist last_twist_;
    nav_msgs::msg::Path::SharedPtr global_path_;
    std::size_t last_target_index_ = 0;

    const double linear_max_vel;
    const double lookahead_distance;
    const double wheelbase_;
    const double caster_max_angle_rad_;
};

}  // namespace path_tracker
