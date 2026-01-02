#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <functional>
#include <memory>

// 角度変換（degree → radian）
static inline double dtor(double deg)
{
  return deg * M_PI / 180.0;
}

// 値の制限
static inline double constrain(double val, double min_val, double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

class CasterController : public rclcpp::Node
{
public:
  CasterController() : Node("caster_controller")
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&CasterController::cmdVelCallback, this, std::placeholders::_1));

    pub_caster_ = create_publisher<std_msgs::msg::Float64>(
      "motor_spin_angle", 10);

    // パラメータ
    wheelbase_ = declare_parameter("wheelbase", 0.5);
    caster_max_angle_ = declare_parameter("caster_max_angle", dtor(30.0));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double linear_vel  = msg->linear.x;
    const double angular_vel = msg->angular.z;

    // === delta生成 ===
    double delta = 0.0;
    if (linear_vel != 0.0) {
      delta = std::asin((wheelbase_ * angular_vel) / linear_vel);
      if (std::isnan(delta)) delta = 0.0;
      delta = constrain(delta, -caster_max_angle_, caster_max_angle_);
    }

    // === motor_pos 補正 ===
    double motor_pos = 0.0;
    if (delta > dtor(1.0)) {
      motor_pos = -1.0 * (delta + dtor(5.0));
    }
    else if (delta < dtor(-1.0)) {
      motor_pos = -1.0 * (delta - dtor(5.0));
    }

    std_msgs::msg::Float64 msg_out;
    msg_out.data = motor_pos;
    pub_caster_->publish(msg_out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_caster_;

  double wheelbase_;
  double caster_max_angle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CasterController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

