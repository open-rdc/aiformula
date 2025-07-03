#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include <torch/torch.h>
#include <torch/script.h>
#include <deque>

#include "chassis_driver/visibility_control.h"

namespace chassis_driver{

class ChassisDriver : public rclcpp::Node {
public:
    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CHASSIS_DRIVER_PUBLIC
    explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS2 Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_stop;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscription_restart;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_rpm;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _subscription_velocity;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_emergency;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_potentio;
    rclcpp::TimerBase::SharedPtr _pub_timer;

    // ROS2 Publishers
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_ref_vel;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_current_state;

    // Callback functions
    void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void _subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg);
    void _subscriber_callback_rpm(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void _subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_potentio(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    void _publisher_callback();
    void send_rpm(const double left_rpm, const double right_rpm);
    
    // RL制御関連
    void initializeRL();
    bool loadRLModel(const std::string& model_path);
    void executeRLControl();
    void resetState();
    
    // RL推論メソッド（新形式対応）
    // 入力: [target_linear, target_angular, current_v, current_w, current_th] 
    // 出力: [left_rpm, right_rpm]
    std::pair<double, double> predictMotorRPM(double cmd_linear, double cmd_angular, 
                                             double obs_linear, double obs_angular, 
                                             double steering_angle);

    rclcpp::QoS _qos = rclcpp::QoS(10);

    // RLモデル関連
    torch::jit::script::Module rl_model_;
    bool rl_model_loaded_ = false;
    torch::Device device_ = torch::kCPU;
    
    // 入力履歴管理（LSTM用）
    std::deque<std::vector<double>> input_history_;
    static constexpr size_t HISTORY_SIZE = 10;

    // 車両パラメータ
    const int interval_ms;
    const double wheel_radius;
    const double tread;
    const double wheelbase;
    const double rotate_ratio;
    const bool is_reverse_left;
    const bool is_reverse_right;

    // RLモデルパラメータ
    const std::string rl_model_file;

    
    // 制約パラメータ
    const double max_linear_vel;
    const double max_angular_vel;

    // 動作モード
    enum class Mode{
        rl_control,     // RL制御モード
        stay,           // 待機モード
        stop            // 停止モード
    } mode = Mode::stay;

    // 速度管理（直接制御）
    double cmd_linear_vel_ = 0.0;
    double cmd_angular_vel_ = 0.0;
    
    // 観測値（vectornavから取得）
    double observed_linear_vel_ = 0.0;
    double observed_angular_vel_ = 0.0;
    
    
    // センサフィードバック
    double current_potentio_ = 0.0;
    
    // 時刻管理
    rclcpp::Time last_update_time_;
};

}  // namespace chassis_driver