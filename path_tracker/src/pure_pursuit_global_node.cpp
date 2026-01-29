#include "path_tracker/pure_pursuit_global_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace path_tracker {

PurePursuitGlobal::PurePursuitGlobal(const rclcpp::NodeOptions& options) : PurePursuitGlobal("", options) {}

PurePursuitGlobal::PurePursuitGlobal(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pure_pursuit_global_node", name_space, options),
  linear_max_vel(get_parameter("linear_max.vel").as_double()),
  lookahead_distance(get_parameter("lookahead_distance").as_double()),
  curvature_gain(get_parameter("curvature_gain").as_double()),
  wheelbase_(get_parameter("wheelbase").as_double()),
  caster_max_angle_rad_(get_parameter("caster_max_angle").as_double() * 0.017453292519943295)
{
    subscription_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_planner/path",
        qos_,
        std::bind(&PurePursuitGlobal::path_callback, this, std::placeholders::_1)
    );
    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body",
        qos_,
        std::bind(&PurePursuitGlobal::velocity_callback, this, std::placeholders::_1)
    );
    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/vectornav/imu",
        qos_,
        std::bind(&PurePursuitGlobal::imu_callback, this, std::placeholders::_1)
    );
    subscription_autonomous_ = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous",
        qos_,
        std::bind(&PurePursuitGlobal::autonomous_callback, this, std::placeholders::_1)
    );
    publisher_vel_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("cmd_vel", qos_);
    publisher_self_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/path_tracker/self_pose",
        qos_
    );
    publisher_target_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/path_tracker/target_pose",
        qos_
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PurePursuitGlobal::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "PurePursuitGlobal node has been initialized. lookahead_distance: %.2f", lookahead_distance);
}

void PurePursuitGlobal::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg){
    autonomous_flag_ = msg->data;
}

void PurePursuitGlobal::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
    if (has_path_ || !msg || msg->poses.empty()) {
        return;
    }

    global_path_ = msg;
    has_path_ = true;
    RCLCPP_INFO(this->get_logger(), "PurePursuitGlobal node received global path");
    if (last_target_index_ >= global_path_->poses.size()) {
        last_target_index_ = 0;
    }

}

void PurePursuitGlobal::velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg){
    if (!msg) {
        return;
    }

    last_twist_ = msg->twist.twist;
    has_velocity_ = true;
}

void PurePursuitGlobal::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    if (!msg) {
        return;
    }

    current_pose_.orientation = msg->orientation;
    has_yaw_reference_ = true;
}

void PurePursuitGlobal::timer_callback(){
    if (!autonomous_flag_ || !global_path_ || global_path_->poses.empty() || !has_velocity_ || !has_yaw_reference_) {
        return;
    }

    constexpr double dt = 0.1;
    const double yaw = yaw_from_quaternion(current_pose_.orientation) - yaw_reference_;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const double vx_map = cos_yaw * last_twist_.linear.x - sin_yaw * last_twist_.linear.y;
    const double vy_map = sin_yaw * last_twist_.linear.x + cos_yaw * last_twist_.linear.y;

    current_pose_.position.x += vx_map * dt;
    current_pose_.position.y += vy_map * dt;

    const double current_x = current_pose_.position.x;
    const double current_y = current_pose_.position.y;

    const auto begin_it = global_path_->poses.begin();
    const auto end_it = global_path_->poses.end();

    auto closest_it = std::min_element(
        begin_it,
        end_it,
        [current_x, current_y](const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
            const double dx_a = a.pose.position.x - current_x;
            const double dy_a = a.pose.position.y - current_y;
            const double dx_b = b.pose.position.x - current_x;
            const double dy_b = b.pose.position.y - current_y;
            return (dx_a * dx_a + dy_a * dy_a) < (dx_b * dx_b + dy_b * dy_b);
        }
    );

    // Start searching from the closest point so std::find_if begins at the target anchor.
    auto target_it = std::find_if(
        closest_it,
        end_it,
        [this, current_x, current_y](const geometry_msgs::msg::PoseStamped& pose) {
            const double dx = pose.pose.position.x - current_x;
            const double dy = pose.pose.position.y - current_y;
            return std::hypot(dx, dy) >= lookahead_distance;
        }
    );

    if (target_it == end_it) {
        target_it = std::prev(end_it);
    }

    last_target_index_ = static_cast<std::size_t>(std::distance(begin_it, target_it));

    const double dx = target_it->pose.position.x - current_x;
    const double dy = target_it->pose.position.y - current_y;

    const double target_x = cos_yaw * dx + sin_yaw * dy;
    const double target_y = -sin_yaw * dx + cos_yaw * dy;
    const double distance = std::hypot(target_x, target_y);

    geometry_msgs::msg::PoseStamped self_pose_msg;
    self_pose_msg.header.stamp = this->now();
    self_pose_msg.header.frame_id = "base_link";
    self_pose_msg.pose = closest_it->pose;

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.stamp = self_pose_msg.header.stamp;
    target_pose_msg.header.frame_id = "base_link";
    target_pose_msg.pose = target_it->pose;

    publisher_self_pose_->publish(self_pose_msg);
    publisher_target_pose_->publish(target_pose_msg);

    ackermann_msgs::msg::AckermannDrive command;

    if (distance < 1e-6) {
        publisher_vel_->publish(command);
        return;
    }

    const double safe_lookahead = std::max(lookahead_distance, 1e-3);
    const double linear_scale = std::clamp(distance / safe_lookahead, 0.0, 1.0);
    const double linear_velocity = std::clamp(linear_max_vel * linear_scale, 0.0, linear_max_vel);

    const double alpha = std::atan2(target_y, target_x);
    const double steer_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);
    const double steer_angle_clamped = std::clamp(steer_angle, -caster_max_angle_rad_, caster_max_angle_rad_);

    command.speed = linear_velocity;
    command.steering_angle = steer_angle_clamped;
    publisher_vel_->publish(command);
}

double PurePursuitGlobal::yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat){
    const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace path_tracker
