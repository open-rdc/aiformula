#include "localization/odom_tf_node.hpp"

#include <cmath>
#include <stdexcept>

namespace localization
{
namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double HALF_PI = PI * 0.5;
}

OdomTfNode::OdomTfNode(const rclcpp::NodeOptions& options)
: OdomTfNode("", options)
{
}

OdomTfNode::OdomTfNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("odom_tf_node", name_space, options),
  publish_period_ms_(get_parameter("publish_period_ms").as_int()),
  odom_frame_id_(get_parameter("odom_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  imu_topic_(get_parameter("imu_topic").as_string()),
  velocity_topic_(get_parameter("velocity_topic").as_string()),
  odom_topic_(get_parameter("odom_topic").as_string()),
  imu_yaw_convention_(get_parameter("imu_yaw_convention").as_string()),
  max_integration_dt_(get_parameter("max_integration_dt").as_double()),
  qos_(rclcpp::QoS(10)),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  initial_imu_yaw_(0.0),
  latest_imu_yaw_(0.0),
  has_initial_imu_yaw_(false),
  has_velocity_stamp_(false),
  has_odom_state_(false),
  latest_velocity_stamp_(0, 0, get_clock()->get_clock_type())
{
    if (publish_period_ms_ <= 0) {
        throw std::invalid_argument("publish_period_ms must be greater than 0");
    }
    if (odom_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("odom_frame_id and base_frame_id must not be empty");
    }
    if (imu_topic_.empty() || velocity_topic_.empty() || odom_topic_.empty()) {
        throw std::invalid_argument("odom_tf_node topic parameters must not be empty");
    }
    if (imu_yaw_convention_ != "heading_north_cw" &&
        imu_yaw_convention_ != "heading_north_ccw" &&
        imu_yaw_convention_ != "ros_enu")
    {
        throw std::invalid_argument(
            "imu_yaw_convention must be heading_north_cw, heading_north_ccw, or ros_enu");
    }
    if (max_integration_dt_ <= 0.0) {
        throw std::invalid_argument("max_integration_dt must be greater than 0");
    }

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        qos_,
        std::bind(&OdomTfNode::imu_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        velocity_topic_,
        qos_,
        std::bind(&OdomTfNode::velocity_callback, this, std::placeholders::_1));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&OdomTfNode::timer_callback, this));
}

void OdomTfNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("IMU message must not be null");
    }
    const double imu_yaw = imu_yaw_to_enu_yaw(yaw_from_quaternion(msg->orientation));
    if (!std::isfinite(imu_yaw)) {
        throw std::runtime_error("IMU yaw is not finite");
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_yaw_ = imu_yaw;
    if (!has_initial_imu_yaw_) {
        initial_imu_yaw_ = imu_yaw;
        yaw_ = 0.0;
        has_initial_imu_yaw_ = true;
    } else {
        yaw_ = normalize_angle(latest_imu_yaw_ - initial_imu_yaw_);
    }
}

void OdomTfNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("velocity message must not be null");
    }
    if (msg->header.frame_id != base_frame_id_ && msg->header.frame_id != "vectornav") {
        throw std::runtime_error(
            "velocity_body frame_id must be " + base_frame_id_ + " or vectornav, got " + msg->header.frame_id);
    }

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_initial_imu_yaw_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "waiting for IMU before integrating odom -> base_link");
        return;
    }
    integrate_velocity(*msg, stamp);
}

void OdomTfNode::integrate_velocity(
    const geometry_msgs::msg::TwistWithCovarianceStamped& velocity_msg,
    const rclcpp::Time& stamp)
{
    if (!std::isfinite(velocity_msg.twist.twist.linear.x) ||
        !std::isfinite(velocity_msg.twist.twist.linear.y) ||
        !std::isfinite(velocity_msg.twist.twist.angular.z))
    {
        throw std::runtime_error("velocity_body twist contains non-finite values");
    }

    latest_twist_ = velocity_msg.twist.twist;
    yaw_ = normalize_angle(latest_imu_yaw_ - initial_imu_yaw_);

    if (!has_velocity_stamp_) {
        latest_velocity_stamp_ = stamp;
        has_velocity_stamp_ = true;
        has_odom_state_ = true;
        return;
    }

    const double dt = (stamp - latest_velocity_stamp_).seconds();
    latest_velocity_stamp_ = stamp;
    if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "velocity_body timestamp did not increase, skipping odom integration");
        return;
    }
    if (dt > max_integration_dt_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "velocity_body dt %.3f exceeds max_integration_dt %.3f, skipping odom integration",
            dt,
            max_integration_dt_);
        return;
    }

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double vx_body = velocity_msg.twist.twist.linear.x;
    const double vy_body = velocity_msg.twist.twist.linear.y;
    x_ += (cos_yaw * vx_body - sin_yaw * vy_body) * dt;
    y_ += (sin_yaw * vx_body + cos_yaw * vy_body) * dt;
    has_odom_state_ = true;
}

void OdomTfNode::timer_callback()
{
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry odometry;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_odom_state_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "waiting for IMU and velocity_body before publishing odom -> base_link");
            return;
        }
        const rclcpp::Time stamp = now();
        transform = make_transform(stamp);
        odometry = make_odometry(stamp);
    }

    tf_broadcaster_->sendTransform(transform);
    odom_publisher_->publish(odometry);
}

geometry_msgs::msg::TransformStamped OdomTfNode::make_transform(const rclcpp::Time& stamp) const
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = yaw_to_quaternion(yaw_);
    return transform;
}

nav_msgs::msg::Odometry OdomTfNode::make_odometry(const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = yaw_to_quaternion(yaw_);
    odometry.twist.twist = latest_twist_;
    return odometry;
}

double OdomTfNode::yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
    const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double OdomTfNode::imu_yaw_to_enu_yaw(const double imu_yaw) const
{
    if (imu_yaw_convention_ == "heading_north_cw") {
        return normalize_angle(HALF_PI - imu_yaw);
    }
    if (imu_yaw_convention_ == "heading_north_ccw") {
        return normalize_angle(HALF_PI + imu_yaw);
    }
    if (imu_yaw_convention_ == "ros_enu") {
        return normalize_angle(imu_yaw);
    }
    throw std::runtime_error("unsupported imu_yaw_convention: " + imu_yaw_convention_);
}

geometry_msgs::msg::Quaternion OdomTfNode::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
    return quaternion;
}

double OdomTfNode::normalize_angle(double angle)
{
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

}  // namespace localization
