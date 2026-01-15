#include "chassis_driver/chassis_driver_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

#include <float.h>
#include <cmath>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int()),
wheel_radius(get_parameter("wheel_radius").as_double()),
tread(get_parameter("tread").as_double()),
wheelbase(get_parameter("wheelbase").as_double()),
rotate_ratio(1.0 / get_parameter("reduction_ratio").as_double()),
is_reverse_left(get_parameter("reverse_left_flag").as_bool()),
is_reverse_right(get_parameter("reverse_right_flag").as_bool()),
caster_max_count(get_parameter("caster_max_count").as_int()),
caster_max_angle(dtor(get_parameter("caster_max_angle").as_double())),
caster_gear_ratio(get_parameter("caster_gear_ratio").as_double()),
caster_wheel_radius(this->get_parameter("caster_wheel_radius").as_double()),

linear_limit(DBL_MAX,
get_parameter("linear_max.vel").as_double(),
get_parameter("linear_max.acc").as_double()),

angular_limit(DBL_MAX,
dtor(get_parameter("angular_max.vel").as_double()),
dtor(get_parameter("angular_max.acc").as_double()))
{
    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_restart, this, std::placeholders::_1)
    );
    _subscription_caster_orientation = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_012",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_caster_orientation, this, std::placeholders::_1)
    );
    _subscription_caster_rotation = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_013",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_caster_rotation, this, std::placeholders::_1)
    );
    _subscription_emergency = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_712",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_emergency, this, std::placeholders::_1)
    );
    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_ref_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);
    publisher_odrive = this->create_publisher<odrive_can::msg::ControlMessage>("/odrive_axis0/control_message", _qos);
    publisher_caster_data = this->create_publisher<std_msgs::msg::Float64MultiArray>("caster_data", _qos);
    publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("caster_odom", _qos);

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
/*従動輪オドメトリ計算*/
    if(caster_rotation_initialized){
        const double delta_rotation = caster_rotation - caster_rotation_prev_for_odom;
        caster_rotation_prev_for_odom = caster_rotation;

        const double delta_travel = delta_rotation * caster_wheel_radius;
        const double delta_theta = delta_travel * std::sin(caster_orientation) / wheelbase;

        const double delta_center = delta_travel * std::cos(caster_orientation);

        const double heading_mid = odom_yaw + delta_theta * 0.5;
        odom_x += delta_center * std::cos(heading_mid);
        odom_y += delta_center * std::sin(heading_mid);
        odom_yaw = normalize_angle(odom_yaw + delta_theta);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "base_link";
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;

        geometry_msgs::msg::Quaternion orientation_msg;
        orientation_msg.x = 0.0;
        orientation_msg.y = 0.0;
        orientation_msg.z = std::sin(odom_yaw * 0.5);
        orientation_msg.w = std::cos(odom_yaw * 0.5);
        odom_msg.pose.pose.orientation = orientation_msg;

        publisher_odom->publish(odom_msg);
    }

/*駆動輪制御*/
    linear_planner.cycle();
    angular_planner.cycle();

    if(mode == Mode::stop || mode == Mode::stay){
        this->send_rpm(0.0, 0.0);
        return;
    }

    // 速度計画機の参照 -> 送信
    const double linear_vel = linear_planner.vel();
    const double angular_vel = angular_planner.vel();
    send_rpm(linear_vel, angular_vel);

    // （デバッグ用）ロボットの目標速度指令値を出版
    auto msg_ref_vel = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg_ref_vel->header.stamp = this->now();
    msg_ref_vel->header.frame_id = "base_link";
    msg_ref_vel->twist.linear.x = linear_vel;
    msg_ref_vel->twist.angular.z = angular_vel;
    publisher_ref_vel->publish(*msg_ref_vel);

/*従動輪制御*/
    // 目標舵角の生成
    double delta = 0.0;
    if(linear_vel == 0.0){
        delta = 0.0;
    }
    else{
        delta = std::asin((wheelbase*angular_vel) / linear_vel);
        if(std::isnan(delta)) delta = 0.0;
        delta = constrain(delta, -caster_max_angle, caster_max_angle);
    }

    // モータ制御
    double motor_pos = 0.0;
    if(delta > dtor(1.0)){
        motor_pos = -1.0 * (delta + dtor(5.0));
    }
    else if(delta < dtor(-1.0)){
        motor_pos = -1.0 * (delta - dtor(5.0));
    }
    // RCLCPP_INFO(this->get_logger(), "DEL:%.2f POS:%.2f ENC:%.2f", rtod(delta), rtod(motor_pos), rtod(caster_orientation));

    // ODriveにトルク指令を送信
    auto msg_odrive_control = std::make_shared<odrive_can::msg::ControlMessage>();
    msg_odrive_control->control_mode = 3;
    msg_odrive_control->input_mode = 1;
    msg_odrive_control->input_pos = motor_pos;
    msg_odrive_control->input_vel = 0.0;
    msg_odrive_control->input_torque = 0.0;
    publisher_odrive->publish(*msg_odrive_control);

    // （テスト用）従動輪{目標舵角，実測舵角，回転位置｝を出版
    std_msgs::msg::Float64MultiArray caster_data_msg;
    caster_data_msg.data = {delta, caster_orientation, caster_rotation};
    publisher_caster_data->publish(caster_data_msg);
}

void ChassisDriver::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;

    velplanner::Physics_t physics_zero(0.0, 0.0, 0.0);
    linear_planner.current(physics_zero);
    angular_planner.current(physics_zero);

    caster_rotation_initialized = false;
    caster_rotation_count = 0;

    caster_rotation_prev_for_odom = 0.0;
    odom_x = 0.0;
    odom_y = 0.0;
    odom_yaw = 0.0;
    RCLCPP_INFO(this->get_logger(), "再起動");
}

void ChassisDriver::_subscriber_callback_caster_orientation(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    const int count = static_cast<int>(bytes_to_int16(_candata));
    caster_orientation = count / static_cast<double>(caster_max_count) * 2.0 * d_pi;
    // RCLCPP_INFO(this->get_logger(), "CAS_ORI:%f CNT:%d", rtod(caster_orientation), count);
}
void ChassisDriver::_subscriber_callback_caster_rotation(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    const int count = static_cast<int>(bytes_to_int16(_candata));
    if(!caster_rotation_initialized){
        caster_rotation_lastcount = count;
        caster_rotation_initialized = true;
    }
    const int count_gap = count - caster_rotation_lastcount;
    caster_rotation_lastcount = count;

    caster_rotation_count += count_gap;
    if(count_gap > caster_max_count / 2) caster_rotation_count -= caster_max_count;
    else if(count_gap < -caster_max_count / 2) caster_rotation_count += caster_max_count;

    // 取付位置からマイナスをかける
    caster_rotation = -caster_rotation_count / static_cast<double>(caster_max_count) * 2.0 * d_pi * caster_gear_ratio;
    // RCLCPP_INFO(this->get_logger(), "CAS_ROT:%f CNT:%d", rtod(caster_rotation), count);
    // RCLCPP_INFO(this->get_logger(), "ROT CRR:%f CNT:%d", rtod(current_rotation), count);
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

double ChassisDriver::normalize_angle(double angle){
    return std::atan2(std::sin(angle), std::cos(angle));
}


}  // namespace chassis_driver
