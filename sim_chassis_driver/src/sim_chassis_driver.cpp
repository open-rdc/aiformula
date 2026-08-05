#include "sim_chassis_driver/sim_chassis_driver.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

#include <cmath>
#include <float.h>

using namespace utils;

namespace sim_chassis_driver {

SimChassisDriver::SimChassisDriver(const rclcpp::NodeOptions& options)
: SimChassisDriver("", options) {}

SimChassisDriver::SimChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("sim_chassis_driver_node", name_space, options),
  interval_ms(get_parameter("interval_ms").as_int()),
  wheel_radius(get_parameter("wheel_radius").as_double()),
  tread(get_parameter("tread").as_double()),
  wheelbase(get_parameter("wheelbase").as_double()),
  caster_wheel_radius(this->get_parameter("caster.wheel_radius").as_double()),
  reel_radius(this->get_parameter("caster.reel_radius").as_double()),
  steering_radius(this->get_parameter("caster.steering_radius").as_double()),
  preload_length(this->get_parameter("caster.preload_length").as_double()),
  preload_gain(this->get_parameter("caster.preload_gain").as_double()),
  linear_limit(DBL_MAX,
               get_parameter("linear_max.vel").as_double(),
               get_parameter("linear_max.acc").as_double()),
  steering_limit(dtor(get_parameter("steering_max.pos").as_double()),
                 DBL_MAX, DBL_MAX),
  drive_pid(get_parameter("interval_ms").as_int())
{
  subscription_vel_ = this->create_subscription<steered_drive_msg::msg::SteeredDrive>(
      "cmd_vel",
      qos_,
      std::bind(&SimChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1));

  subscription_restart_ = this->create_subscription<std_msgs::msg::Empty>(
      "restart",
      qos_,
      std::bind(&SimChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1));

  subscription_caster_orientation_ = this->create_subscription<std_msgs::msg::Float64>(
      "caster_orientation",
      qos_,
      std::bind(&SimChassisDriver::_subscriber_callback_caster_orientation, this, std::placeholders::_1));

  subscription_bodyvel_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "vectornav/velocity_body",
      qos_,
      std::bind(&SimChassisDriver::_subscriber_callback_bodyvel, this, std::placeholders::_1));

  publisher_diff_drive_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_twists", qos_);
  publisher_reel_position_ = this->create_publisher<std_msgs::msg::Float64>("caster_reel_position_cmd", qos_);
  publisher_caster_data_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("caster_data", qos_);

  pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      [this]() { _publisher_callback(); });

  linear_planner.limit(linear_limit);
  drive_pid.gain(
      get_parameter("drive_pid.p_gain").as_double(),
      get_parameter("drive_pid.i_gain").as_double(),
      get_parameter("drive_pid.d_gain").as_double());

  RCLCPP_INFO(this->get_logger(),
              "Sim Chassis Driver Node Started. max vel: %.2f m/s, steering angle: %.1f deg",
              linear_limit.vel, rtod(steering_limit.pos));
}

void SimChassisDriver::_subscriber_callback_vel(const steered_drive_msg::msg::SteeredDrive::SharedPtr msg)
{
  
  if (mode == Mode::stop) {
    return;
  }
  mode = Mode::cmd;
  std::cout << "SimChassisDriver: Received velocity command." << std::endl;

  const double linear_vel = constrain(msg->velocity, -linear_limit.vel, linear_limit.vel);
  cmd_steering = constrain(msg->steering_angle, -steering_limit.pos, steering_limit.pos);
  linear_planner.vel(linear_vel);
}

void SimChassisDriver::_publisher_callback()
{
  linear_planner.cycle();
  const double linear_vel = linear_planner.vel();

  if (mode == Mode::stop || mode == Mode::stay) {
    std::cout << "SimChassisDriver: stop or stay mode, sending zero command." << std::endl;
    send_sim_command(0.0, 0.0);
    return;
  }

  double delta = 0.0;
  bool driving_flag = false;
  if (std::abs(linear_vel) > 0.1) {
    delta = cmd_steering;
    driving_flag = true;
  }
  const double body_vel_squared = current_body_vel.linear.x * current_body_vel.linear.x;

  double motor_pos = 0.0;
  bool straight_flag = false;
  double winding_length = std::abs(steering_radius * std::sin(caster_orientation)) * preload_gain * body_vel_squared;
  if (std::abs(delta) < dtor(1.0) && driving_flag) {
    straight_flag = true;
    winding_length = preload_length;
  }
  winding_length = constrain(winding_length, 0.0, preload_length);
  motor_pos = winding_length / reel_radius;

  std_msgs::msg::Float64 reel_pos_msg;
  reel_pos_msg.data = motor_pos;
  publisher_reel_position_->publish(reel_pos_msg);

  std_msgs::msg::Float64MultiArray caster_data_msg;
  caster_data_msg.data = {delta, caster_orientation, winding_length};
  publisher_caster_data_->publish(caster_data_msg);

  const double angular_command = (straight_flag ? 0.0 : 1.0) * drive_pid.cycle(caster_orientation, delta) * body_vel_squared;
  send_sim_command(linear_vel, angular_command);
}

void SimChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr)
{
  mode = Mode::stay;
  velplanner::Physics_t physics_zero(0.0, 0.0, 0.0);
  linear_planner.current(physics_zero);
  RCLCPP_INFO(this->get_logger(), "Sim Chassis Driver restarted.");
}

void SimChassisDriver::_subscriber_callback_caster_orientation(const std_msgs::msg::Float64::SharedPtr msg)
{
  caster_orientation = msg->data;
}

void SimChassisDriver::_subscriber_callback_bodyvel(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  current_body_vel = msg->twist.twist;
}

void SimChassisDriver::send_sim_command(const double linear_vel, const double angular_vel)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear_vel;
  twist_msg.angular.z = angular_vel;
  publisher_diff_drive_->publish(twist_msg);
}

double SimChassisDriver::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace sim_chassis_driver