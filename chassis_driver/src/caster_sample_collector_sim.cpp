#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <aiformula_msgs/msg/caster_sample.hpp>

#include <limits>
#include <mutex>

class CasterSampleCollectorSim : public rclcpp::Node
{
public:
  CasterSampleCollectorSim()
  : Node("caster_sample_collector_sim")
  {
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    out_topic_ = declare_parameter<std::string>("out_topic", "caster_data_sim");
    domain_ = declare_parameter<int>("domain", 1);
    run_id_ = declare_parameter<int>("run_id", 0);

    v_ = 0.0;
    w_ = 0.0;
    caster_ = nan();
    motor_cmd_ = nan();
    delta_ = nan();

    got_cmd_ = false;

    auto qos = rclcpp::QoS(10).best_effort();

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        v_ = msg->linear.x;
        w_ = msg->angular.z;
        got_cmd_ = true;
      });

    sub_caster_ = create_subscription<sensor_msgs::msg::JointState>(
      "caster_yaw_angle", qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "caster_yaw_wheel") {
	    if (i < msg->position.size())
              caster_ = msg->position[i];
            break;
          }
        }
      });

    sub_motor_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "motor_spin_angle", qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        motor_cmd_ = msg->data;
      });
    
    sub_caster_data_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "caster_data", qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (msg->data.size() >= 1) {
          delta_ = msg->data[0];   // 目標角[rad]
        }
        // msg->data[1] は caster_orientation（今回 simでは不要なら使わない）
      });

    pub_ = create_publisher<aiformula_msgs::msg::CasterSample>(out_topic_, 10);

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CasterSampleCollectorSim::on_timer, this));

    RCLCPP_INFO(get_logger(), "CasterSampleCollectorSim started");
  }

private:
  static double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  void on_timer()
  {
    if (!got_cmd_) return;

    aiformula_msgs::msg::CasterSample msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "sim";
    msg.v = v_;
    msg.w = w_;
    msg.caster = caster_;
    msg.motor_cmd = motor_cmd_;
    msg.delta = delta_;
    msg.domain = static_cast<uint8_t>(domain_);
    msg.run_id = run_id_;

    pub_->publish(msg);
  }

  double rate_hz_;
  std::string out_topic_;
  int domain_;
  int run_id_;

  std::mutex mtx_;
  double v_, w_;
  double caster_;
  double motor_cmd_;
  double delta_;
  bool got_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_caster_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_motor_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_caster_data_;
  rclcpp::Publisher<aiformula_msgs::msg::CasterSample>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CasterSampleCollectorSim>());
  rclcpp::shutdown();
  return 0;
}

