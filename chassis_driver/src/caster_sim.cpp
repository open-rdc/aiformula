#include "chassis_driver/caster_sim_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <float.h>

using namespace utils;

namespace chassis_driver {

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions &options)
    : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string &name_space,
                             const rclcpp::NodeOptions &options)
    : rclcpp::Node("chassis_driver_node", name_space, options) {

  // ==================================================
  // パラメータ宣言（完全版）
  // ==================================================
  this->declare_parameter("interval_ms", 2);

  this->declare_parameter("wheel_radius", 0.124);
  this->declare_parameter("tread", 0.6);
  this->declare_parameter("wheelbase", 0.65);
  this->declare_parameter("reduction_ratio", 1.0);

  this->declare_parameter("reverse_left_flag", false);
  this->declare_parameter("reverse_right_flag", false);

  this->declare_parameter("caster_max_angle", 15.0); // [deg]
  this->declare_parameter("caster_max_count", 4096);

  this->declare_parameter("linear_max.vel", 3.0);
  this->declare_parameter("linear_max.acc", 5.0);
  this->declare_parameter("linear_max.jer", 0.0);

  this->declare_parameter("angular_max.vel", 90.0);
  this->declare_parameter("angular_max.acc", 240.0);
  this->declare_parameter("angular_max.jer", 0.0);

  this->declare_parameter("autonomous_flag", false);

  // ==================================================
  // パラメータ取得
  // ==================================================
  interval_ms = this->get_parameter("interval_ms").as_int();
  wheel_radius = this->get_parameter("wheel_radius").as_double();
  tread = this->get_parameter("tread").as_double();
  wheelbase = this->get_parameter("wheelbase").as_double();
  rotate_ratio = 1.0 / this->get_parameter("reduction_ratio").as_double();

  is_reverse_left = this->get_parameter("reverse_left_flag").as_bool();
  is_reverse_right = this->get_parameter("reverse_right_flag").as_bool();

  caster_max_angle = dtor(this->get_parameter("caster_max_angle").as_double());
  caster_max_count = this->get_parameter("caster_max_count").as_int();

  linear_limit = velplanner::Limit(
      DBL_MAX, this->get_parameter("linear_max.vel").as_double(),
      this->get_parameter("linear_max.acc").as_double());

  angular_limit = velplanner::Limit(
      DBL_MAX, dtor(this->get_parameter("angular_max.vel").as_double()),
      dtor(this->get_parameter("angular_max.acc").as_double()));

  _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", _qos,
      std::bind(&ChassisDriver::_subscriber_callback_vel, this,
                std::placeholders::_1));
  _subscription_caster_yaw =
      this->create_subscription<sensor_msgs::msg::JointState>(
          "caster_yaw_angle",
          rclcpp::SensorDataQoS(), // まずはこれが無難（BestEffort想定）
          std::bind(&ChassisDriver::_subscriber_callback_caster_yaw, this,
                    std::placeholders::_1));

  publisher_ref_vel =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);
  publisher_caster =
      this->create_publisher<std_msgs::msg::Float64>("motor_spin_angle", _qos);
  publisher_caster_data =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("caster_data",
                                                               _qos);

  _pub_timer = this->create_wall_timer(std::chrono::milliseconds(interval_ms),
                                       [this] { _publisher_callback(); });

  linear_planner.limit(linear_limit);
  angular_planner.limit(angular_limit);

  RCLCPP_INFO(this->get_logger(),
              "Chassis Driver Node has been started. max vel: %.2f m/s, max "
              "angular vel: %.2f deg/s",
              linear_limit.vel, rtod(angular_limit.vel));
}

void ChassisDriver::_subscriber_callback_vel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {

  const double linear_vel =
      constrain(msg->linear.x, -linear_limit.vel, linear_limit.vel);
  const double angular_vel =
      constrain(msg->angular.z, -angular_limit.vel, angular_limit.vel);
  linear_planner.vel(linear_vel);
  angular_planner.vel(angular_vel);
}

void ChassisDriver::_publisher_callback() {
  linear_planner.cycle();
  angular_planner.cycle();

  // 速度計画機の参照
  const double linear_vel = linear_planner.vel();
  const double angular_vel = angular_planner.vel();

  // 従動輪目標角度の生成
  double delta = 0.0;
  if (linear_vel == 0.0) {
    delta = 0.0;
  } else {
    delta = std::asin((wheelbase * angular_vel) / linear_vel);
    if (std::isnan(delta))
      delta = 0.0;
    delta = constrain(delta, -caster_max_angle, caster_max_angle);
  }

  // （テスト用）従動輪目標角度の出版
  std_msgs::msg::Float64MultiArray caster_data_msg;
  caster_data_msg.data = {delta, caster_orientation};
  publisher_caster_data->publish(caster_data_msg);

  // モータ制御
  double motor_pos = 0.0;
  if (delta > dtor(1.0)) {
    motor_pos = -1.0 * (delta + dtor(5.0));
  } else if (delta < dtor(-1.0)) {
    motor_pos = -1.0 * (delta - dtor(5.0));
  }
  // RCLCPP_INFO(this->get_logger(), "DEL:%.2f POS:%.2f ENC:%.2f", rtod(delta),
  // rtod(motor_pos), rtod(caster_orientation));

  std_msgs::msg::Float64 msg_out;
  msg_out.data = motor_pos;

  publisher_caster->publish(msg_out);

  // デバッグ用にロボットの目標速度指令値を出版
  auto msg_ref_vel = std::make_shared<geometry_msgs::msg::TwistStamped>();
  msg_ref_vel->header.stamp = this->now();
  msg_ref_vel->header.frame_id = "map";
  msg_ref_vel->twist.linear.x = linear_vel;
  msg_ref_vel->twist.angular.z = angular_vel;
  publisher_ref_vel->publish(*msg_ref_vel);
}

void ChassisDriver::_subscriber_callback_caster_yaw(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  static const std::string kCasterJointName = "caster_yaw_wheel";

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == kCasterJointName) {
      if (i < msg->position.size()) {
        caster_orientation = msg->position[i];
        caster_orientation = std::atan2(std::sin(caster_orientation),
                                        std::cos(caster_orientation));
      }
      return;
    }
  }
}

} // namespace chassis_driver

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<chassis_driver::ChassisDriver>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
