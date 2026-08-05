#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "steered_drive_msg/msg/steered_drive.hpp"
#include "base/velplanner.hpp"
#include "utilities/position_pid.hpp"
#include "sim_chassis_driver/visibility_control.h"

namespace sim_chassis_driver {

class SimChassisDriver : public rclcpp::Node {
public:
  SIM_CHASSIS_DRIVER_PUBLIC
  explicit SimChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  SIM_CHASSIS_DRIVER_PUBLIC
  explicit SimChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<steered_drive_msg::msg::SteeredDrive>::SharedPtr subscription_vel_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_restart_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_caster_orientation_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_bodyvel_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  void _subscriber_callback_vel(const steered_drive_msg::msg::SteeredDrive::SharedPtr msg);
  void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
  void _subscriber_callback_caster_orientation(const std_msgs::msg::Float64::SharedPtr msg);
  void _subscriber_callback_bodyvel(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void _publisher_callback();
  void send_sim_command(const double linear_vel, const double angular_vel);
  static double normalize_angle(double angle);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_diff_drive_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_reel_position_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_caster_data_;

  rclcpp::QoS qos_ = rclcpp::QoS(10);

  const int interval_ms;
  const double wheel_radius;
  const double tread;
  const double wheelbase;
  const double caster_wheel_radius;
  const double reel_radius;
  const double steering_radius;
  const double preload_length;
  const double preload_gain;

  velplanner::VelPlanner linear_planner;
  const velplanner::Limit linear_limit;
  const velplanner::Limit steering_limit;
  controller::PositionPid drive_pid;

  double cmd_steering = 0.0;
  double caster_orientation = 0.0;
  geometry_msgs::msg::Twist current_body_vel;

  enum class Mode {
    cmd,
    stay,
    stop
  } mode = Mode::stop;
};

}  // namespace sim_chassis_driver
