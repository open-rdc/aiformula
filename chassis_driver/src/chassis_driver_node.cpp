#include "chassis_driver/chassis_driver_node.hpp"
#include <float.h>
#include <cmath>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Utility functions
template<typename T>
T constrain(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

void int_to_bytes(uint8_t* bytes, int value) {
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
}

int bytes_to_int(const uint8_t* bytes) {
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

const double d_pi = M_PI;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int()),
wheel_radius(get_parameter("wheel_radius").as_double()),
tread(get_parameter("tread").as_double()),
wheelbase(get_parameter("wheelbase").as_double()),
rotate_ratio(get_parameter("reduction_ratio").as_double()),
is_reverse_left(get_parameter("reverse_left_flag").as_bool()),
is_reverse_right(get_parameter("reverse_right_flag").as_bool()),
rl_model_file(get_parameter("rl_model_file").as_string()),
max_linear_vel(get_parameter("linear_max.vel").as_double()),
max_angular_vel(get_parameter("angular_max.vel").as_double())
{
    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1)
    );
    _subscription_rpm = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_711", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_rpm, this, std::placeholders::_1)
    );
    _subscription_velocity = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/vectornav/velocity", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_velocity, this, std::placeholders::_1)
    );
    _subscription_emergency = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_712", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_emergency, this, std::placeholders::_1)
    );
    _subscription_potentio = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_011", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_potentio, this, std::placeholders::_1)
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_ref_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);
    publisher_current_state = this->create_publisher<geometry_msgs::msg::TwistStamped>("current_state", _qos);

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    // RL制御初期化
    initializeRL();
    
    // 時刻初期化
    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "ChassisDriver initialized with RL control");
}

void ChassisDriver::initializeRL() {
    // デバイス自動選択
    device_ = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    
    // 状態を初期値にリセット
    resetState();
    
    std::string rl_model_path = ament_index_cpp::get_package_share_directory("chassis_driver") + "/weights/" + rl_model_file;
    if (!loadRLModel(rl_model_path)) {
        throw std::runtime_error("RLモデルの読み込みに失敗しました: " + rl_model_path);
    }
    
    RCLCPP_INFO(this->get_logger(), "RL制御初期化完了 - デバイス: %s", 
                device_.is_cuda() ? "CUDA" : "CPU");
}

bool ChassisDriver::loadRLModel(const std::string& model_path) {
    try {
        rl_model_ = torch::jit::load(model_path, device_);
        rl_model_.eval();
        
        rl_model_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "RLモデル読み込み成功: %s", model_path.c_str());
        return true;
        
    } catch (const std::exception& e) {
        rl_model_loaded_ = false;
        RCLCPP_WARN(this->get_logger(), "RLモデル読み込み失敗: %s - Error: %s", 
                     model_path.c_str(), e.what());
        return false;
    }
}

void ChassisDriver::resetState() {
    cmd_linear_vel_ = 0.0;
    cmd_angular_vel_ = 0.0;
    observed_linear_vel_ = 0.0;
    observed_angular_vel_ = 0.0;
    current_potentio_ = 0.0;
    input_history_.clear();
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (mode == Mode::stop) return;
    
    cmd_linear_vel_ = constrain(msg->linear.x, -max_linear_vel, max_linear_vel);
    cmd_angular_vel_ = constrain(msg->angular.z, -max_angular_vel, max_angular_vel);
    
    mode = Mode::rl_control;
    
    RCLCPP_DEBUG(this->get_logger(), "速度指令受信: linear=%f, angular=%f", 
                 cmd_linear_vel_, cmd_angular_vel_);
}

void ChassisDriver::_subscriber_callback_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    observed_linear_vel_ = msg->twist.linear.x;
    observed_angular_vel_ = msg->twist.angular.z;
    
    RCLCPP_DEBUG(this->get_logger(), "観測値更新: linear=%f, angular=%f", 
                 observed_linear_vel_, observed_angular_vel_);
}

void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr /* msg */) {
    mode = Mode::stop;
    cmd_linear_vel_ = 0.0;
    cmd_angular_vel_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "停止モード");
}

void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr /* msg */) {
    mode = Mode::stay;
    resetState();
    RCLCPP_INFO(this->get_logger(), "再稼働 - 待機モード");
}

void ChassisDriver::_subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    if (msg->candlc >= 7) {
        uint8_t emergency_flag = msg->candata[6];
        if (emergency_flag && mode != Mode::stop) {
            mode = Mode::stop;
            cmd_linear_vel_ = 0.0;
            cmd_angular_vel_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "緊急停止信号受信!");
        }
    }
}

void ChassisDriver::_subscriber_callback_potentio(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    if (msg->candlc >= 4) {
        uint8_t candata_array[4];
        for (int i = 0; i < 4; ++i) {
            candata_array[i] = msg->candata[i];
        }
        int32_t potentio_raw = bytes_to_int(candata_array);
        current_potentio_ = static_cast<double>(potentio_raw);
        
        RCLCPP_DEBUG(this->get_logger(), "ポテンショメータ値更新: %d", static_cast<int>(current_potentio_));
    }
}

void ChassisDriver::_publisher_callback() {
    if (mode == Mode::stop || mode == Mode::stay) {
        send_rpm(0.0, 0.0);
        
        auto state_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        state_msg->header.stamp = this->now();
        state_msg->header.frame_id = "base_link";
        state_msg->twist.linear.x = 0.0;
        state_msg->twist.angular.z = 0.0;
        publisher_current_state->publish(*state_msg);
        return;
    }
    
    if (mode == Mode::rl_control) {
        executeRLControl();
    }
    
    // 現在状態をパブリッシュ
    auto state_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    state_msg->header.stamp = this->now();
    state_msg->header.frame_id = "base_link";
    state_msg->twist.linear.x = observed_linear_vel_;
    state_msg->twist.angular.z = observed_angular_vel_;
    publisher_current_state->publish(*state_msg);
}

void ChassisDriver::executeRLControl() {
    // ステアリング角度を計算（度からラジアンに変換）
    double steering_angle = static_cast<double>(current_potentio_);
    
    auto [left_rpm, right_rpm] = predictMotorRPM(cmd_linear_vel_, cmd_angular_vel_, 
                                                 observed_linear_vel_, observed_angular_vel_, 
                                                 steering_angle);
    
    // モーター制御実行
    send_rpm(left_rpm, right_rpm);
    
    auto ref_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    ref_vel_msg->header.stamp = this->now();
    ref_vel_msg->header.frame_id = "base_link";
    ref_vel_msg->twist.linear.x = cmd_linear_vel_;
    ref_vel_msg->twist.angular.z = cmd_angular_vel_;
    publisher_ref_vel->publish(*ref_vel_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "RL制御 - 推定RPM: [%.2f, %.2f]", left_rpm, right_rpm);
}

std::pair<double, double> ChassisDriver::predictMotorRPM(double cmd_linear, double cmd_angular, 
                                                       double obs_linear, double obs_angular, 
                                                       double steering_angle) {
    // RLモデル用の入力形式: [target_linear, target_angular, current_v, current_w, current_th]
    torch::Tensor input_tensor = torch::zeros({1, 5}, 
                                             torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    
    input_tensor[0][0] = cmd_linear;      // target_linear
    input_tensor[0][1] = cmd_angular;     // target_angular
    input_tensor[0][2] = obs_linear;      // current_v
    input_tensor[0][3] = obs_angular;     // current_w
    input_tensor[0][4] = steering_angle;  // current_th (ポテンショメータ値)
    
    // 推論実行
    torch::NoGradGuard no_grad;
    std::vector<torch::jit::IValue> inputs{input_tensor};
    torch::Tensor output = rl_model_.forward(inputs).toTensor();
    
    // RLモデルの出力処理（2次元出力: left_rpm, right_rpm）
    auto output_cpu = output.to(torch::kCPU);
    auto output_data = output_cpu.accessor<float, 2>();
    
    double left_rpm = static_cast<double>(output_data[0][0]);
    double right_rpm = static_cast<double>(output_data[0][1]);
    
    // RPM制限
    const double max_rpm = 700.0;
    left_rpm = constrain(left_rpm, -max_rpm, max_rpm);
    right_rpm = constrain(right_rpm, -max_rpm, max_rpm);
    
    RCLCPP_DEBUG(this->get_logger(), "RLモデル推論完了: 入力[%.3f,%.3f,%.3f,%.3f,%.1f] → 出力[%.1f,%.1f]", 
                 cmd_linear, cmd_angular, obs_linear, obs_angular, steering_angle, left_rpm, right_rpm);
    
    return {left_rpm, right_rpm};
}

void ChassisDriver::send_rpm(const double left_rpm, const double right_rpm) {
    // 回転方向制御
    const double corrected_left_rpm = (is_reverse_left ? -1 : 1) * left_rpm;
    const double corrected_right_rpm = (is_reverse_right ? -1 : 1) * right_rpm;
    
    // CAN送信
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = 0x210;
    msg_can->candlc = 8;

    uint8_t _candata[8];
    int_to_bytes(_candata, static_cast<int>(corrected_right_rpm));
    int_to_bytes(_candata + 4, static_cast<int>(corrected_left_rpm));

    for (int i = 0; i < msg_can->candlc; i++) {
        msg_can->candata[i] = _candata[i];
    }
    publisher_can->publish(*msg_can);

    RCLCPP_DEBUG(this->get_logger(), "直接RPM送信: left=%f, right=%f", 
                 corrected_left_rpm, corrected_right_rpm);
}


}  // namespace chassis_driver