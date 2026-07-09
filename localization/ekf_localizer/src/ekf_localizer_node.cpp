#include "ekf_localizer/ekf_localizer_node.hpp"

#include <cmath>

#include "ekf_localizer/delay_gate.hpp"
#include "utilities/utils.hpp"

namespace ekf_localizer
{
namespace
{

EkfLocalizerConfig make_ekf_config(rclcpp::Node& node)
{
    EkfLocalizerConfig config;
    config.initial_position_variance = node.get_parameter("initial_position_variance").as_double();
    config.initial_yaw_variance = node.get_parameter("initial_yaw_variance").as_double();
    config.process_position_variance = node.get_parameter("process_position_variance").as_double();
    config.process_yaw_variance = node.get_parameter("process_yaw_variance").as_double();
    config.process_velocity_variance = node.get_parameter("process_velocity_variance").as_double();
    config.process_yaw_rate_variance = node.get_parameter("process_yaw_rate_variance").as_double();
    config.position_gate_dist = node.get_parameter("position_gate_dist").as_double();
    config.yaw_gate_dist = node.get_parameter("yaw_gate_dist").as_double();
    return config;
}

}

EkfLocalizerNode::EkfLocalizerNode(const rclcpp::NodeOptions& options)
: EkfLocalizerNode("", options)
{
}

EkfLocalizerNode::EkfLocalizerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("ekf_localizer_node", name_space, options),
  input_timeout_s_(get_parameter("input_timeout_s").as_double()),
  predict_interval_ms_(get_parameter("predict_interval_ms").as_int()),
  tf_interval_ms_(get_parameter("tf_interval_ms").as_int()),
  icp_pose_additional_delay_s_(get_parameter("icp_pose_additional_delay_s").as_double()),
  icp_pose_max_delay_s_(get_parameter("icp_pose_max_delay_s").as_double()),
  ekf_config_(make_ekf_config(*this)),
  ekf_localizer_(ekf_config_),
  has_icp_pose_stamp_(false),
  last_icp_pose_stamp_(0, 0, get_clock()->get_clock_type()),
  has_velocity_(false),
  latest_velocity_(0.0),
  latest_yaw_rate_(0.0)
{
    icp_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/icp_pose", rclcpp::QoS(1),
        std::bind(&EkfLocalizerNode::icp_pose_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&EkfLocalizerNode::velocity_callback, this, std::placeholders::_1));

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose", rclcpp::QoS(1));

    predict_timer_ = create_wall_timer(
        std::chrono::milliseconds(predict_interval_ms_),
        std::bind(&EkfLocalizerNode::predict_timer_callback, this));
    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(tf_interval_ms_),
        std::bind(&EkfLocalizerNode::tf_timer_callback, this));
}

void EkfLocalizerNode::icp_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = utils::yaw_from_quaternion(msg->pose.pose.orientation);
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());

    Eigen::Matrix2d position_covariance;
    position_covariance(0, 0) = msg->pose.covariance[0];
    position_covariance(0, 1) = msg->pose.covariance[1];
    position_covariance(1, 0) = msg->pose.covariance[6];
    position_covariance(1, 1) = msg->pose.covariance[7];
    const double yaw_variance = msg->pose.covariance[35];

    std::lock_guard<std::mutex> lock(state_mutex_);
    has_icp_pose_stamp_ = true;
    last_icp_pose_stamp_ = stamp;

    if (!ekf_localizer_.initialized()) {
        ekf_localizer_.initialize(x, y, yaw, stamp);
        return;
    }

    const DelayGateResult delay_gate = check_delay_gate(
        get_clock()->now(), stamp, icp_pose_additional_delay_s_, icp_pose_max_delay_s_);
    if (!delay_gate.passed) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "icp_poseの遅延%.3fsが上限%.3fsを超えたためdelay gateで棄却する",
            delay_gate.delay_time_s, icp_pose_max_delay_s_);
        return;
    }

    try {
        if (!ekf_localizer_.update_position(x, y, position_covariance, stamp)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "icp_pose position update rejected by Mahalanobis gate (%.2f, %.2f)", x, y);
        }
        if (!ekf_localizer_.update_yaw(yaw, yaw_variance, stamp)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "icp_pose yaw update rejected by Mahalanobis gate (%.3f rad)", yaw);
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "EKF update skipped: %s", error.what());
    }
}

void EkfLocalizerNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.angular.z)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "velocity_bodyのtwistに非有限値が含まれるため無視する");
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    has_velocity_ = true;
    latest_velocity_ = msg->twist.twist.linear.x;
    latest_yaw_rate_ = msg->twist.twist.angular.z;
}

void EkfLocalizerNode::predict_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!ekf_localizer_.initialized() || !has_velocity_) {
        return;
    }

    const rclcpp::Time stamp = get_clock()->now();
    try {
        ekf_localizer_.predict(latest_velocity_, latest_yaw_rate_, stamp);
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "EKF predict skipped: %s", error.what());
    }
}

void EkfLocalizerNode::tf_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!ekf_localizer_.initialized()) {
        return;
    }

    const rclcpp::Time now_time = get_clock()->now();
    if (!has_icp_pose_stamp_ || (now_time - last_icp_pose_stamp_).seconds() > input_timeout_s_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "icp_pose が %.1fs 以上途絶しているため自己位置のpublishを停止する", input_timeout_s_);
        return;
    }
    pose_publisher_->publish(ekf_localizer_.make_pose("map"));
}


}
