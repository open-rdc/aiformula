#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/velplanner.hpp"

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
    void _subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_potentio(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    void _publisher_callback();
    void send_rpm(const double linear_vel, const double angular_vel);
    
    // MPPI制御関連
    void initializeMPPI();
    bool loadMPPIModel(const std::string& model_path);
    void updateStateFromModel(double control_linear, double control_angular);
    void resetState();

    rclcpp::QoS _qos = rclcpp::QoS(10);

    // MPPI統合制御プランナー
    velplanner::VelPlanner mppi_planner_;
    bool model_loaded_ = false;

    // 車両パラメータ
    const int interval_ms;
    const double wheel_radius;
    const double tread;
    const double wheelbase;
    const double rotate_ratio;
    const bool is_reverse_left;
    const bool is_reverse_right;

    // MPPIパラメータ
    const int num_samples;
    const int horizon;
    const double lambda;
    const double noise_sigma_linear;
    const double noise_sigma_angular;
    const double weight_vel_error;
    const double weight_control_error;
    const double weight_smoothness;
    
    // 制約パラメータ
    const double max_linear_vel;
    const double max_angular_vel;

    // 動作モード
    enum class Mode{
        mppi_control,   // MPPI制御モード
        stay,           // 待機モード
        stop            // 停止モード
    } mode = Mode::stay;

    // 現在の状態（モデル予測による状態管理）
    velplanner::State_t current_state_;
    
    // 目標速度
    double target_linear_vel_ = 0.0;
    double target_angular_vel_ = 0.0;
    
    // センサフィードバック
    double current_potentio_ = 0.0;
    
    // 時刻管理
    rclcpp::Time last_update_time_;
    const double dt_ = 0.05;  // MPPIの時間ステップと合わせる
};

}  // namespace chassis_driver