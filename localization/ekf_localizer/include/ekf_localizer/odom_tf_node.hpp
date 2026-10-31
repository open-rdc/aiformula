#pragma once

#include "ekf_localizer/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>

namespace ekf_localizer {

class OdomTfNode : public rclcpp::Node {
   public:
    EKF_LOCALIZER_PUBLIC
    explicit OdomTfNode (const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

    EKF_LOCALIZER_PUBLIC
    explicit OdomTfNode (const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

   private:
    void imu_callback (const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void velocity_callback (const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
    void timer_callback ();

    void                                 integrate_velocity (const geometry_msgs::msg::TwistWithCovarianceStamped &velocity_msg, const rclcpp::Time &stamp);
    geometry_msgs::msg::TransformStamped make_transform (const rclcpp::Time &stamp) const;
    nav_msgs::msg::Odometry              make_odometry (const rclcpp::Time &stamp) const;

    const int         publish_period_ms_;
    const double      max_integration_dt_;
    const rclcpp::QoS qos_;

    double                    x_;
    double                    y_;
    double                    yaw_;
    double                    initial_imu_yaw_;
    double                    latest_imu_yaw_;
    bool                      has_initial_imu_yaw_;
    bool                      has_velocity_stamp_;
    bool                      has_odom_state_;
    rclcpp::Time              latest_velocity_stamp_;
    geometry_msgs::msg::Twist latest_twist_;
    mutable std::mutex        data_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                          imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                           odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                  tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                                    timer_;
};

}  // namespace ekf_localizer
