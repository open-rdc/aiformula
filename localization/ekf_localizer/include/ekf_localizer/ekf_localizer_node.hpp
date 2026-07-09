#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/ekf_localizer.hpp"
#include "ekf_localizer/velocity_gate.hpp"
#include "ekf_localizer/visibility_control.h"

namespace ekf_localizer
{

class EkfLocalizerNode : public rclcpp::Node
{
public:
    EKF_LOCALIZER_PUBLIC
    explicit EkfLocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    EKF_LOCALIZER_PUBLIC
    explicit EkfLocalizerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void icp_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void predict_timer_callback();
    void tf_timer_callback();

    const double input_timeout_s_;
    const int predict_interval_ms_;
    const int tf_interval_ms_;
    const double icp_pose_additional_delay_s_;
    const double icp_pose_max_delay_s_;
    const double velocity_additional_delay_s_;
    const double velocity_max_delay_s_;
    EkfLocalizerConfig ekf_config_;
    EkfLocalizer ekf_localizer_;
    VelocityGate velocity_gate_;

    bool has_icp_pose_stamp_;
    rclcpp::Time last_icp_pose_stamp_;
    bool has_velocity_;
    double latest_velocity_;
    double latest_yaw_rate_;
    bool has_velocity_stamp_;
    rclcpp::Time last_velocity_stamp_;
    mutable std::mutex state_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr icp_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr predict_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

}
