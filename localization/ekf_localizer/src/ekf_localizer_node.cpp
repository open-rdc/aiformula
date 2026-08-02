#include "ekf_localizer/ekf_localizer_node.hpp"

#include <cmath>

#include "utilities/utils.hpp"
#include "utilities/vectornav_frame.hpp"

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
    config.min_position_variance = node.get_parameter("min_position_variance").as_double();
    config.min_yaw_variance = node.get_parameter("min_yaw_variance").as_double();
    config.position_gate_max_reject_duration_s =
        node.get_parameter("position_gate_max_reject_duration_s").as_double();
    config.yaw_gate_max_reject_duration_s =
        node.get_parameter("yaw_gate_max_reject_duration_s").as_double();
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
  predict_interval_ms_(get_parameter("predict_interval_ms").as_int()),
  tf_interval_ms_(get_parameter("tf_interval_ms").as_int()),
  pf_pose_max_delay_s_(get_parameter("pf_pose_max_delay_s").as_double()),
  velocity_max_delay_s_(get_parameter("velocity_max_delay_s").as_double()),
  ekf_config_(make_ekf_config(*this)),
  ekf_localizer_(ekf_config_),
  velocity_gate_(
        ekf_config_.process_velocity_variance,
        ekf_config_.process_yaw_rate_variance,
      get_parameter("velocity_gate_dist").as_double(),
      get_parameter("min_velocity_variance").as_double(),
      get_parameter("min_yaw_rate_variance").as_double(),
      get_parameter("velocity_gate_max_reject_duration_s").as_double()),
    last_pf_pose_stamp_(0, 0, get_clock()->get_clock_type()),
    last_velocity_stamp_(0, 0, get_clock()->get_clock_type())
{
    pf_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pf_pose", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&EkfLocalizerNode::pf_pose_callback, this, std::placeholders::_1));
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

void EkfLocalizerNode::pf_pose_callback(
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

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pf_poseに非有限値が含まれるため無視する");
        return;
    }
    if (!position_covariance.allFinite() || position_covariance(0, 0) <= 0.0 ||
        position_covariance(1, 1) <= 0.0 || !std::isfinite(yaw_variance) || yaw_variance <= 0.0)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pf_poseの共分散が不正なため無視する");
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    has_pf_pose_stamp_ = true;
    last_pf_pose_stamp_ = stamp;

    if (!ekf_localizer_.initialized()) {
        ekf_localizer_.initialize(x, y, yaw, stamp);
        return;
    }

    const DelayGateResult delay_gate = check_delay_gate(
        get_clock()->now(), stamp, pf_pose_max_delay_s_);
    if (!delay_gate.passed) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pf_poseの遅延%.3fsが上限%.3fsを超えたためdelay gateで棄却する",
            delay_gate.delay_time_s, pf_pose_max_delay_s_);
        return;
    }

    if (!ekf_localizer_.update_position(x, y, position_covariance, stamp)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pf_pose position update rejected by Mahalanobis gate (%.2f, %.2f)", x, y);
    }
    if (!ekf_localizer_.update_yaw(yaw, yaw_variance, stamp)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pf_pose yaw update rejected by Mahalanobis gate (%.3f rad)", yaw);
    }
}

void EkfLocalizerNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    // velocity_bodyはVN body系(x前 / y右 / z下)で来るのでREP-103へ直す。
    const geometry_msgs::msg::Twist twist = utils::vn_body_to_rep103(msg->twist.twist);

    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "velocity_bodyのtwistに非有限値が含まれるため無視する");
        return;
    }

    const double velocity_variance = msg->twist.covariance[0];
    const double yaw_rate_variance = msg->twist.covariance[35];
    if (!std::isfinite(velocity_variance) || !std::isfinite(yaw_rate_variance))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "velocity_bodyの共分散が不正なため無視する");
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());

    std::lock_guard<std::mutex> lock(state_mutex_);

    const DelayGateResult delay_gate = check_delay_gate(
        get_clock()->now(), stamp, velocity_max_delay_s_);
    if (!delay_gate.passed) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "velocity_bodyの遅延%.3fsが上限%.3fsを超えたためdelay gateで棄却する",
            delay_gate.delay_time_s, velocity_max_delay_s_);
        return;
    }

    const double dt = has_velocity_stamp_ ?
        std::max((stamp - last_velocity_stamp_).seconds(), 0.0) : 0.0;
    last_velocity_stamp_ = stamp;
    has_velocity_stamp_ = true;

    const VelocityGateResult gate_result = velocity_gate_.update(
        twist.linear.x, twist.angular.z,
        velocity_variance, yaw_rate_variance, dt);
    if (!gate_result.passed) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "velocity_bodyがmahalanobis gateで外れ値として棄却された (v=%.2f, yaw_rate=%.2f)",
            twist.linear.x, twist.angular.z);
        return;
    }

    has_velocity_ = true;
    latest_velocity_ = gate_result.velocity;
    latest_yaw_rate_ = gate_result.yaw_rate;
}

void EkfLocalizerNode::predict_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!ekf_localizer_.initialized() || !has_velocity_) {
        return;
    }
    ekf_localizer_.predict(latest_velocity_, latest_yaw_rate_, get_clock()->now());
}

void EkfLocalizerNode::tf_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!ekf_localizer_.initialized()) {
        return;
    }
    pose_publisher_->publish(ekf_localizer_.make_pose("map"));
}


}
