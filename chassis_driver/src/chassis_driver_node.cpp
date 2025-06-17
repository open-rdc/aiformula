#include "chassis_driver/chassis_driver_node.hpp"
#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"
#include <float.h>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
interval_ms(this->declare_parameter("interval_ms", 50)),
wheel_radius(this->declare_parameter("wheel_radius", 0.095)),
tread(this->declare_parameter("tread", 0.35)),
wheelbase(this->declare_parameter("wheelbase", 0.28)),
rotate_ratio(1.0 / this->declare_parameter("reduction_ratio", 4.0)),
is_reverse_left(this->declare_parameter("reverse_left_flag", false)),
is_reverse_right(this->declare_parameter("reverse_right_flag", true)),
max_linear_vel(this->declare_parameter("max_linear_vel", 2.0)),
max_angular_vel(this->declare_parameter("max_angular_vel", 2.0)),

num_samples(this->declare_parameter("num_samples", 100)),
horizon(this->declare_parameter("horizon", 20)),
lambda(this->declare_parameter("lambda", 1.0)),
noise_sigma_linear(this->declare_parameter("noise_sigma_linear", 0.3)),
noise_sigma_angular(this->declare_parameter("noise_sigma_angular", 0.5)),
weight_vel_error(this->declare_parameter("weight_vel_error", 1.0)),
weight_control_error(this->declare_parameter("weight_control_error", 0.1)),
weight_smoothness(this->declare_parameter("weight_smoothness", 0.05))
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
    _subscription_emergency = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_712", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_emergency, this, std::placeholders::_1)
    );
    _subscription_potentio = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_11", _qos,
        std::bind(&ChassisDriver::_subscriber_callback_potentio, this, std::placeholders::_1)
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_ref_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);
    publisher_current_state = this->create_publisher<geometry_msgs::msg::TwistStamped>("current_state", _qos);

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    // MPPI制御初期化
    initializeMPPI();
    
    // 時刻初期化
    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "ChassisDriver initialized with MPPI control");
}

void ChassisDriver::initializeMPPI() {
    // 状態を初期値にリセット
    resetState();
    
    // MPPIパラメータの設定
    velplanner::MPPIParams params;
    params.num_samples = num_samples;
    params.horizon = horizon;
    params.lambda = lambda;
    params.noise_sigma_linear = noise_sigma_linear;
    params.noise_sigma_angular = noise_sigma_angular;
    params.weight_vel_error = weight_vel_error;
    params.weight_control_effort = weight_control_error;
    params.weight_smoothness = weight_smoothness;
    params.dt = interval_ms;
    
    mppi_planner_.setMPPIParams(params);
    std::string model_path = ament_index_cpp::get_package_share_directory("main_executor") + "/weights/predict.pt",
    
    loadMPPIModel(model_path);
}

bool ChassisDriver::loadMPPIModel(const std::string& model_path) {
    if (mppi_planner_.loadModel(model_path)) {
        model_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "MPPI model loaded successfully: %s", model_path.c_str());
        return true;
    } else {
        model_loaded_ = false;
        RCLCPP_ERROR(this->get_logger(), "Failed to load MPPI model: %s", model_path.c_str());
        return false;
    }
}

void ChassisDriver::resetState() {
    // 状態を全て0で初期化
    current_state_.linear_vel = 0.0;
    current_state_.angular_vel = 0.0;
    current_state_.acc_linear_vel = 0.0;
    current_state_.acc_angular_vel = 0.0;
    current_state_.potentio = 0;
    
    target_linear_vel_ = 0.0;
    target_angular_vel_ = 0.0;
    current_potentio_ = 0.0;
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (mode == Mode::stop) return;
    
    // 目標速度の設定（制約適用）
    target_linear_vel_ = constrain(msg->linear.x, -max_linear_vel, max_linear_vel);
    target_angular_vel_ = constrain(msg->angular.z, -max_angular_vel, max_angular_vel);
    
    // MPPIプランナーに目標速度を設定
    mppi_planner_.setTargetVelocity(target_linear_vel_, target_angular_vel_);
    
    // 制御モードに遷移
    mode = Mode::mppi_control;
    
    RCLCPP_DEBUG(this->get_logger(), "目標速度設定: linear=%f, angular=%f", 
                 target_linear_vel_, target_angular_vel_);
}

void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;  // 未使用パラメータ警告抑制
    mode = Mode::stop;
    target_linear_vel_ = 0.0;
    target_angular_vel_ = 0.0;
    mppi_planner_.setTargetVelocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "停止モード");
}

void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;  // 未使用パラメータ警告抑制
    mode = Mode::stay;
    resetState();
    target_linear_vel_ = 0.0;
    target_angular_vel_ = 0.0;
    mppi_planner_.setTargetVelocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "再稼働 - 待機モード");
}

void ChassisDriver::_subscriber_callback_rpm(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    // RPMフィードバック処理（将来の拡張用）
    (void)msg;  // 未使用パラメータ警告抑制
}

void ChassisDriver::_subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    if (msg->candlc >= 7) {
        uint8_t emergency_flag = msg->candata[6];
        if (emergency_flag && mode != Mode::stop) {
            mode = Mode::stop;
            target_linear_vel_ = 0.0;
            target_angular_vel_ = 0.0;
            mppi_planner_.setTargetVelocity(0.0, 0.0);
            RCLCPP_WARN(this->get_logger(), "緊急停止信号受信!");
        }
    }
}

void ChassisDriver::_subscriber_callback_potentio(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    if (msg->candlc >= 4) {
        // ポテンショメータ値をCANデータから取得
        uint8_t candata_array[4];
        for (int i = 0; i < 4; ++i) {
            candata_array[i] = msg->candata[i];
        }
        int32_t potentio_raw = bytes_to_int(candata_array);
        current_potentio_ = static_cast<double>(potentio_raw);
        
        // 現在状態のポテンショメータ値を更新
        current_state_.potentio = static_cast<int>(current_potentio_);
    }
}

void ChassisDriver::_publisher_callback() {
    if (mode == Mode::stop || mode == Mode::stay) {
        // 停止または待機時は速度0で送信
        send_rpm(0.0, 0.0);
        
        // 現在状態をパブリッシュ
        auto state_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        state_msg->header.stamp = this->now();
        state_msg->header.frame_id = "base_link";
        state_msg->twist.linear.x = current_state_.linear_vel;
        state_msg->twist.angular.z = current_state_.angular_vel;
        publisher_current_state->publish(*state_msg);
        return;
    }
    
    if (model_loaded_ && mode == Mode::mppi_control) {
        // MPPI制御実行
        mppi_planner_.cycle(current_state_);
        
        // 最適制御入力を取得
        auto [optimal_linear, optimal_angular] = mppi_planner_.getOptimalControl();
        
        // 参照速度として出力（デバッグ用）
        auto ref_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        ref_vel_msg->header.stamp = this->now();
        ref_vel_msg->header.frame_id = "base_link";
        ref_vel_msg->twist.linear.x = optimal_linear;
        ref_vel_msg->twist.angular.z = optimal_angular;
        publisher_ref_vel->publish(*ref_vel_msg);
        
        // モータ制御実行
        send_rpm(optimal_linear, optimal_angular);
        
        // 状態を更新（モデル予測による次状態を使用）
        updateStateFromModel(optimal_linear, optimal_angular);
        
        // 現在状態をパブリッシュ
        auto state_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        state_msg->header.stamp = this->now();
        state_msg->header.frame_id = "base_link";
        state_msg->twist.linear.x = current_state_.linear_vel;
        state_msg->twist.angular.z = current_state_.angular_vel;
        publisher_current_state->publish(*state_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "MPPI制御 - 出力: [%f, %f], 状態: [%f, %f, %d]", 
                     optimal_linear, optimal_angular, 
                     current_state_.linear_vel, current_state_.angular_vel, current_state_.potentio);
    } else {
        // モデル未読み込み時は停止
        send_rpm(0.0, 0.0);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "MPPIモデルが読み込まれていません。制御を停止します。");
    }
}

void ChassisDriver::updateStateFromModel(double control_linear, double control_angular) {
    if (model_loaded_) {
        // MPPI プランナーの状態推定モデルを使用して次状態を予測
        velplanner::State_t predicted_state = mppi_planner_.predictNextState(current_state_, control_linear, control_angular);
        
        // 予測された状態を現在状態として更新
        current_state_.linear_vel = predicted_state.linear_vel;
        current_state_.angular_vel = predicted_state.angular_vel;
        current_state_.acc_linear_vel = predicted_state.acc_linear_vel;
        current_state_.acc_angular_vel = predicted_state.acc_angular_vel;
        
        // ポテンショメータ値の更新
        // センサーフィードバックが利用可能な場合はそれを優先、
        // そうでなければモデル予測値を使用
        if (current_potentio_ != 0.0) {
            // センサーフィードバック優先
            current_state_.potentio = static_cast<int>(current_potentio_);
        } else {
            // モデル予測値を使用
            current_state_.potentio = predicted_state.potentio;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "モデル予測による状態更新: [%f, %f, %f, %f, %d]", 
                     current_state_.linear_vel, current_state_.angular_vel,
                     current_state_.acc_linear_vel, current_state_.acc_angular_vel, 
                     current_state_.potentio);
    }
    
    last_update_time_ = this->now();
}

void ChassisDriver::send_rpm(const double linear_vel, const double angular_vel) {
    // 差動二輪の運動学変換
    const double left_vel = (-tread * angular_vel + 2.0 * linear_vel) / (2.0 * wheel_radius);
    const double right_vel = (tread * angular_vel + 2.0 * linear_vel) / (2.0 * wheel_radius);

    // rad/s -> rpm & 回転方向制御
    const double left_rpm = (is_reverse_left ? -1 : 1) * (left_vel * 30.0 / d_pi) * rotate_ratio;
    const double right_rpm = (is_reverse_right ? -1 : 1) * (right_vel * 30.0 / d_pi) * rotate_ratio;

    // CAN送信
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = 0x210;
    msg_can->candlc = 8;

    uint8_t _candata[8];
    int_to_bytes(_candata, static_cast<int>(right_rpm));
    int_to_bytes(_candata + 4, static_cast<int>(left_rpm));

    for (int i = 0; i < msg_can->candlc; i++) {
        msg_can->candata[i] = _candata[i];
    }
    publisher_can->publish(*msg_can);

    RCLCPP_DEBUG(this->get_logger(), "RPM送信: left=%f, right=%f", left_rpm, right_rpm);
}

}  // namespace chassis_driver