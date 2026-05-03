#include "vectormap_localization/map_odom_tf_node.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace vectormap_localization
{
namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace

MapOdomTfNode::MapOdomTfNode(const rclcpp::NodeOptions& options)
: MapOdomTfNode("", options)
{
}

MapOdomTfNode::MapOdomTfNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("map_odom_tf_node", name_space, options),
  publish_period_ms_(get_parameter("publish_period_ms").as_int()),
  stale_warn_timeout_s_(get_parameter("stale_warn_timeout_s").as_double()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  odom_frame_id_(get_parameter("odom_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  localized_pose_topic_(get_parameter("localized_pose_topic").as_string()),
  qos_(rclcpp::QoS(10)),
  last_update_time_(0, 0, get_clock()->get_clock_type())
{
    if (publish_period_ms_ <= 0) {
        throw std::invalid_argument("publish_period_ms must be greater than 0");
    }
    if (stale_warn_timeout_s_ <= 0.0) {
        throw std::invalid_argument("stale_warn_timeout_s must be greater than 0");
    }
    if (map_frame_id_.empty() || odom_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("frame_id parameters must not be empty");
    }
    if (localized_pose_topic_.empty()) {
        throw std::invalid_argument("localized_pose_topic must not be empty");
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        localized_pose_topic_,
        qos_,
        std::bind(&MapOdomTfNode::localized_pose_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&MapOdomTfNode::timer_callback, this));
}

void MapOdomTfNode::localized_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (msg->header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "localized pose frame_id must be " + map_frame_id_ +
            ", got " + msg->header.frame_id);
    }

    // odom -> base_link の最新 TF を取得
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
        odom_to_base = tf_buffer_->lookupTransform(
            odom_frame_id_,
            base_frame_id_,
            tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "could not look up %s -> %s: %s",
            odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
        return;
    }

    // T_map_base: map フレームにおける base_link の姿勢
    const double x_m   = msg->pose.pose.position.x;
    const double y_m   = msg->pose.pose.position.y;
    const double yaw_m = yaw_from_quaternion(msg->pose.pose.orientation);

    // T_odom_base: odom フレームにおける base_link の姿勢
    const double x_o   = odom_to_base.transform.translation.x;
    const double y_o   = odom_to_base.transform.translation.y;
    const double yaw_o = yaw_from_quaternion(odom_to_base.transform.rotation);

    // T_map_odom = T_map_base * inv(T_odom_base)
    const double yaw_mo = yaw_m - yaw_o;
    const double cos_mo = std::cos(yaw_mo);
    const double sin_mo = std::sin(yaw_mo);

    geometry_msgs::msg::TransformStamped new_transform;
    new_transform.header.frame_id = map_frame_id_;
    new_transform.child_frame_id  = odom_frame_id_;
    new_transform.transform.translation.x = x_m - (cos_mo * x_o - sin_mo * y_o);
    new_transform.transform.translation.y = y_m - (sin_mo * x_o + cos_mo * y_o);
    new_transform.transform.translation.z = 0.0;
    new_transform.transform.rotation = yaw_to_quaternion(yaw_mo);

    std::lock_guard<std::mutex> lock(cache_mutex_);
    cached_transform_    = new_transform;
    has_cached_transform_ = true;
    last_update_time_    = now();
}

void MapOdomTfNode::timer_callback()
{
    geometry_msgs::msg::TransformStamped transform;
    bool has_cache;
    double age_s;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        has_cache = has_cached_transform_;
        if (!has_cache) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "map -> odom TF not yet available: waiting for first localization result");
            return;
        }
        age_s     = (now() - last_update_time_).seconds();
        transform = cached_transform_;
    }

    if (age_s > stale_warn_timeout_s_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "map -> odom TF is stale (%.1f s since last localization update)", age_s);
    }

    // スタンプを現在時刻に更新して tf2 バッファの期限切れを防ぐ
    transform.header.stamp = now();
    tf_broadcaster_->sendTransform(transform);
}

}  // namespace vectormap_localization
