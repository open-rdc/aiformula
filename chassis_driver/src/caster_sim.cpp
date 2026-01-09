#include "chassis_driver/caster_sim_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

#include <float.h>
#include <cmath>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options){


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

    this->declare_parameter("caster_max_angle", 15.0);   // [deg]
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
    interval_ms      = this->get_parameter("interval_ms").as_int();
    wheel_radius     = this->get_parameter("wheel_radius").as_double();
    tread            = this->get_parameter("tread").as_double();
    wheelbase        = this->get_parameter("wheelbase").as_double();
    rotate_ratio     = 1.0 / this->get_parameter("reduction_ratio").as_double();

    is_reverse_left  = this->get_parameter("reverse_left_flag").as_bool();
    is_reverse_right = this->get_parameter("reverse_right_flag").as_bool();

    caster_max_angle = dtor(this->get_parameter("caster_max_angle").as_double());
    caster_max_count = this->get_parameter("caster_max_count").as_int();

    linear_limit = velplanner::Limit(
        DBL_MAX,
        this->get_parameter("linear_max.vel").as_double(),
        this->get_parameter("linear_max.acc").as_double()
    );

    angular_limit = velplanner::Limit(
        DBL_MAX,
        dtor(this->get_parameter("angular_max.vel").as_double()),
        dtor(this->get_parameter("angular_max.acc").as_double())
    );

    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1)
    );
    _subscription_caster = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_012",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_caster, this, std::placeholders::_1)
    );
    _subscription_emergency = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_712",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_emergency, this, std::placeholders::_1)
    );
    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_ref_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);
    publisher_caster = this->create_publisher<std_msgs::msg::Float64>("motor_spin_angle", _qos);
    publisher_caster_data = this->create_publisher<std_msgs::msg::Float64MultiArray>("caster_data", _qos);

    // ODriveのAxis Stateサービスクライアント作成
    odrive_axis_client_ = this->create_client<odrive_can::srv::AxisState>("/odrive_axis0/request_axis_state");

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    linear_planner.limit(linear_limit);
    angular_planner.limit(angular_limit);

    // ODriveのAxis Stateをクローズドループに設定（axis_requested_state: 8）
    auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
    request->axis_requested_state = 8;
    auto future = odrive_axis_client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Chassis Driver Node has been started. max vel: %.2f m/s, max angular vel: %.2f deg/s",
        linear_limit.vel, rtod(angular_limit.vel));
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;
    mode = Mode::cmd;

    const double linear_vel = constrain(msg->linear.x, -linear_limit.vel, linear_limit.vel);
    const double angular_vel = constrain(msg->angular.z, -angular_limit.vel, angular_limit.vel);
    linear_planner.vel(linear_vel);
    angular_planner.vel(angular_vel);
}

void ChassisDriver::_publisher_callback(){
    linear_planner.cycle();
    angular_planner.cycle();

    if(mode == Mode::stop || mode == Mode::stay){
        this->send_rpm(0.0, 0.0);
        return;
    }

    // 速度計画機の参照
    const double linear_vel = linear_planner.vel();
    const double angular_vel = angular_planner.vel();

    // 従動輪目標角度の生成
    double delta = 0.0;
    if(linear_vel == 0.0){
        delta = 0.0;
    }
    else{
        delta = std::asin((wheelbase*angular_vel) / linear_vel);
        if(std::isnan(delta)) delta = 0.0;
        delta = constrain(delta, -caster_max_angle, caster_max_angle);
    }

    // （テスト用）従動輪目標角度の出版
    std_msgs::msg::Float64MultiArray caster_data_msg;
    caster_data_msg.data = {delta, caster_orientation};
    publisher_caster_data->publish(caster_data_msg);

    // モータ制御
    double motor_pos = 0.0;
    if(delta > dtor(1.0)){
        motor_pos = -1.0 * (delta + dtor(5.0));
    }
    else if(delta < dtor(-1.0)){
        motor_pos = -1.0 * (delta - dtor(5.0));
    }
    // RCLCPP_INFO(this->get_logger(), "DEL:%.2f POS:%.2f ENC:%.2f", rtod(delta), rtod(motor_pos), rtod(caster_orientation));

    send_rpm(linear_vel, angular_vel);

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

void ChassisDriver::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;
    RCLCPP_INFO(this->get_logger(), "停止");
}
void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;

    velplanner::Physics_t physics_zero(0.0, 0.0, 0.0);
    linear_planner.current(physics_zero);
    angular_planner.current(physics_zero);
    RCLCPP_INFO(this->get_logger(), "再起動");
}

void ChassisDriver::_subscriber_callback_caster(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    const int count = static_cast<int>(bytes_to_int16(_candata));
    caster_orientation = count / static_cast<double>(caster_max_count) * 2.0 * d_pi;
    // RCLCPP_INFO(this->get_logger(), "CAS:%f CNT:%d", rtod(caster_orientation), count);
}
void ChassisDriver::_subscriber_callback_emergency(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    if(_candata[6] and mode!=Mode::stop){
        mode = Mode::stop;
        RCLCPP_INFO(this->get_logger(), "緊急停止!");
    }
}

void ChassisDriver::send_rpm(const double linear_vel, const double angular_vel){

    // 駆動輪の目標角速度
    const double left_vel = (-tread*angular_vel + 2.0*linear_vel) / (2.0*wheel_radius);
    const double right_vel = (tread*angular_vel + 2.0*linear_vel) / (2.0*wheel_radius);

    // rad/s -> rpm  &  回転方向制御
    const double left_rpm = (is_reverse_left ? -1 : 1) * (left_vel*30.0 / d_pi) * rotate_ratio;
    const double right_rpm = (is_reverse_right ? -1 : 1) * (right_vel*30.0 / d_pi) * rotate_ratio;

    // RCLCPP_INFO(this->get_logger(), "right:%f  left:%f", right_rpm, left_rpm);
    // 出版
    auto msg_can = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_can->canid = 0x210;
    msg_can->candlc = 8;

    uint8_t _candata[8];
    int32_to_bytes(_candata, static_cast<int32_t>(right_rpm));
    int32_to_bytes(_candata+4, static_cast<int32_t>(left_rpm));

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
    publisher_can->publish(*msg_can);

}


}  // namespace chassis_driver


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // ノードのインスタンスを作成
  auto node = std::make_shared<chassis_driver::ChassisDriver>(rclcpp::NodeOptions());
  // 実行（スピン）
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
