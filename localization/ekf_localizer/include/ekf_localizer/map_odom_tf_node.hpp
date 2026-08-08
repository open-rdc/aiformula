#pragma once

#include "ekf_localizer/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace ekf_localizer {

class MapOdomTfNode : public rclcpp::Node {
   public:
    EKF_LOCALIZER_PUBLIC
    explicit MapOdomTfNode (const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

    EKF_LOCALIZER_PUBLIC
    explicit MapOdomTfNode (const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

   private:
    void localized_pose_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timer_callback ();

    const int         publish_period_ms_;
    const double      stale_warn_timeout_s_;
    const rclcpp::QoS qos_;

    geometry_msgs::msg::TransformStamped cached_transform_;
    bool                                 has_cached_transform_{false};
    rclcpp::Time                         last_update_time_;
    mutable std::mutex                   cache_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                 tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>                                               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                                    tf_listener_;
    rclcpp::TimerBase::SharedPtr                                                   timer_;
};

}  // namespace ekf_localizer
