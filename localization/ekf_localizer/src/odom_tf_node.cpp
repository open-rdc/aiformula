#include "ekf_localizer/odom_tf_node.hpp"

#include "utilities/utils.hpp"
#include "utilities/vectornav_frame.hpp"

#include <cmath>

namespace ekf_localizer {
namespace {
constexpr double PI      = 3.14159265358979323846;
constexpr double HALF_PI = PI * 0.5;

double normalize_angle (double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}
}  // namespace

OdomTfNode::OdomTfNode (const rclcpp::NodeOptions &options) : OdomTfNode ("", options) {}

OdomTfNode::OdomTfNode (const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node ("odom_tf_node", name_space, options),
      publish_period_ms_ (get_parameter ("publish_period_ms").as_int ()),
      max_integration_dt_ (get_parameter ("max_integration_dt").as_double ()),
      qos_ (rclcpp::QoS (10)),
      x_ (0.0),
      y_ (0.0),
      yaw_ (0.0),
      initial_imu_yaw_ (0.0),
      latest_imu_yaw_ (0.0),
      has_initial_imu_yaw_ (false),
      has_velocity_stamp_ (false),
      has_odom_state_ (false),
      latest_velocity_stamp_ (0, 0, get_clock ()->get_clock_type ()) {
    imu_subscription_      = create_subscription<sensor_msgs::msg::Imu> ("/vectornav/imu", qos_, std::bind (&OdomTfNode::imu_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped> ("/vectornav/velocity_body", qos_, std::bind (&OdomTfNode::velocity_callback, this, std::placeholders::_1));
    odom_publisher_        = create_publisher<nav_msgs::msg::Odometry> ("/localization/odom", qos_);
    tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster> (*this);
    timer_                 = create_wall_timer (std::chrono::milliseconds (publish_period_ms_), std::bind (&OdomTfNode::timer_callback, this));
}

void OdomTfNode::imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double imu_yaw = normalize_angle (HALF_PI + utils::yaw_from_quaternion (msg->orientation));
    if (!std::isfinite (imu_yaw)) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 1000, "IMUのyawが非有限値のため無視する");
        return;
    }

    std::lock_guard<std::mutex> lock (data_mutex_);
    latest_imu_yaw_ = imu_yaw;
    if (!has_initial_imu_yaw_) {
        initial_imu_yaw_     = imu_yaw;
        yaw_                 = 0.0;
        has_initial_imu_yaw_ = true;
    } else {
        yaw_ = normalize_angle (latest_imu_yaw_ - initial_imu_yaw_);
    }
}

void OdomTfNode::velocity_callback (const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    const rclcpp::Time          stamp (msg->header.stamp, get_clock ()->get_clock_type ());
    std::lock_guard<std::mutex> lock (data_mutex_);
    if (!has_initial_imu_yaw_) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 2000, "waiting for IMU before integrating odom -> base_link");
        return;
    }
    integrate_velocity (*msg, stamp);
}

void OdomTfNode::integrate_velocity (const geometry_msgs::msg::TwistWithCovarianceStamped &velocity_msg, const rclcpp::Time &stamp) {
    // velocity_bodyはVN body系(x前 / y右 / z下)で来るのでREP-103へ直す。
    const geometry_msgs::msg::Twist twist = utils::vn_body_to_rep103 (velocity_msg.twist.twist);

    if (!std::isfinite (twist.linear.x) || !std::isfinite (twist.linear.y) || !std::isfinite (twist.angular.z)) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 1000, "velocity_bodyのtwistに非有限値が含まれるため積分をスキップする");
        return;
    }

    latest_twist_ = twist;
    yaw_          = normalize_angle (latest_imu_yaw_ - initial_imu_yaw_);

    if (!has_velocity_stamp_) {
        latest_velocity_stamp_ = stamp;
        has_velocity_stamp_    = true;
        has_odom_state_        = true;
        return;
    }

    const double dt        = (stamp - latest_velocity_stamp_).seconds ();
    latest_velocity_stamp_ = stamp;
    if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 1000, "velocity_body timestamp did not increase, skipping odom integration");
        return;
    }
    if (dt > max_integration_dt_) {
        RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 1000, "velocity_body dt %.3f exceeds max_integration_dt %.3f, skipping odom integration", dt, max_integration_dt_);
        return;
    }

    const double cos_yaw = std::cos (yaw_);
    const double sin_yaw = std::sin (yaw_);
    const double vx_body = twist.linear.x;
    const double vy_body = twist.linear.y;
    x_ += (cos_yaw * vx_body - sin_yaw * vy_body) * dt;
    y_ += (sin_yaw * vx_body + cos_yaw * vy_body) * dt;
    has_odom_state_ = true;
}

void OdomTfNode::timer_callback () {
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry              odometry;
    {
        std::lock_guard<std::mutex> lock (data_mutex_);
        if (!has_odom_state_) {
            RCLCPP_WARN_THROTTLE (get_logger (), *get_clock (), 2000, "waiting for IMU and velocity_body before publishing odom -> base_link");
            return;
        }
        const rclcpp::Time stamp = now ();
        transform                = make_transform (stamp);
        odometry                 = make_odometry (stamp);
    }

    tf_broadcaster_->sendTransform (transform);
    odom_publisher_->publish (odometry);
}

geometry_msgs::msg::TransformStamped OdomTfNode::make_transform (const rclcpp::Time &stamp) const {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp            = stamp;
    transform.header.frame_id         = "odom";
    transform.child_frame_id          = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation      = utils::yaw_to_quaternion (yaw_);
    return transform;
}

nav_msgs::msg::Odometry OdomTfNode::make_odometry (const rclcpp::Time &stamp) const {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp          = stamp;
    odometry.header.frame_id       = "odom";
    odometry.child_frame_id        = "base_link";
    odometry.pose.pose.position.x  = x_;
    odometry.pose.pose.position.y  = y_;
    odometry.pose.pose.position.z  = 0.0;
    odometry.pose.pose.orientation = utils::yaw_to_quaternion (yaw_);
    odometry.twist.twist           = latest_twist_;
    return odometry;
}

}  // namespace ekf_localizer
