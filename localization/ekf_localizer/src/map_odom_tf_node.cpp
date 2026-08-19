#include "ekf_localizer/map_odom_tf_node.hpp"

#include "utilities/utils.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <cmath>

namespace ekf_localizer {

MapOdomTfNode::MapOdomTfNode (const rclcpp::NodeOptions &options) : MapOdomTfNode ("", options) {}

MapOdomTfNode::MapOdomTfNode (const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node ("map_odom_tf_node", name_space, options),
      publish_period_ms_ (get_parameter ("publish_period_ms").as_int ()),
      stale_warn_timeout_s_ (get_parameter ("stale_warn_timeout_s").as_double ()),
      qos_ (rclcpp::QoS (10)),
      last_update_time_ (0, 0, get_clock ()->get_clock_type ()) {
    tf_buffer_      = std::make_shared<tf2_ros::Buffer> (get_clock ());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster> (*this);

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped> ("/localization/pose", qos_, std::bind (&MapOdomTfNode::localized_pose_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer (std::chrono::milliseconds (publish_period_ms_), std::bind (&MapOdomTfNode::timer_callback, this));
}

void MapOdomTfNode::localized_pose_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
        odom_to_base = tf_buffer_->lookupTransform ("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 2000, "could not look up %s -> %s: %s", "odom", "base_link", ex.what ());
        return;
    }

    const double x_m   = msg->pose.pose.position.x;
    const double y_m   = msg->pose.pose.position.y;
    const double yaw_m = utils::yaw_from_quaternion (msg->pose.pose.orientation);

    const double x_o   = odom_to_base.transform.translation.x;
    const double y_o   = odom_to_base.transform.translation.y;
    const double yaw_o = utils::yaw_from_quaternion (odom_to_base.transform.rotation);

    const double yaw_mo = yaw_m - yaw_o;
    const double cos_mo = std::cos (yaw_mo);
    const double sin_mo = std::sin (yaw_mo);

    geometry_msgs::msg::TransformStamped new_transform;
    new_transform.header.frame_id         = "map";
    new_transform.child_frame_id          = "odom";
    new_transform.transform.translation.x = x_m - (cos_mo * x_o - sin_mo * y_o);
    new_transform.transform.translation.y = y_m - (sin_mo * x_o + cos_mo * y_o);
    new_transform.transform.translation.z = 0.0;
    new_transform.transform.rotation      = utils::yaw_to_quaternion (yaw_mo);

    std::lock_guard<std::mutex> lock (cache_mutex_);
    cached_transform_     = new_transform;
    has_cached_transform_ = true;
    last_update_time_     = now ();
}

void MapOdomTfNode::timer_callback () {
    geometry_msgs::msg::TransformStamped transform;
    bool                                 has_cache;
    double                               age_s;
    {
        std::lock_guard<std::mutex> lock (cache_mutex_);
        has_cache = has_cached_transform_;
        if (!has_cache) {
            RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 2000, "map -> odom TF not yet available: waiting for first localization result");
            return;
        }
        age_s     = (now () - last_update_time_).seconds ();
        transform = cached_transform_;
    }

    if (age_s > stale_warn_timeout_s_) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 2000, "map -> odom TF is stale (%.1f s since last localization update)", age_s);
    }

    transform.header.stamp = now ();
    tf_broadcaster_->sendTransform (transform);
}

}  // namespace ekf_localizer
