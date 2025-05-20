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

linear_limit(DBL_MAX,
get_parameter("linear_max.vel").as_double(),
get_parameter("linear_max.acc").as_double()),

angular_limit(DBL_MAX,
get_parameter("angular_max.vel").as_double(),
get_parameter("angular_max.acc").as_double())
{
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
    _subscription_rpm = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_711",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_rpm, this, std::placeholders::_1)
    );
    _subscription_emergency = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_712",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_emergency, this, std::placeholders::_1)
    );
    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_steer = this->create_publisher<std_msgs::msg::Float64>("cybergear/pos", _qos);
    publisher_ref_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_vel", _qos);

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    linear_planner.limit(linear_limit);
    angular_planner.limit(angular_limit);
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

    const double linear_vel = linear_planner.vel();
    const double angular_vel = angular_planner.vel();
    send_rpm(linear_vel, angular_vel);

    // デバッグ用にロボットの目標速度指令値を出版
    auto msg_ref_vel = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg_ref_vel->header.stamp = this->now();
    msg_ref_vel->header.frame_id = "map";
    msg_ref_vel->twist.linear.x = linear_vel;
    msg_ref_vel->twist.angular.z = angular_vel;
    publisher_ref_vel->publish(*msg_ref_vel);

    // 従動輪角度の送信
    auto msg_steer = std::make_shared<std_msgs::msg::Float64>();
    if(linear_vel == 0.0){
        msg_steer->data = 0.0;
    }
    else{
        msg_steer->data = std::asin((wheelbase*angular_vel) / linear_vel);
        if(std::isnan(msg_steer->data)) msg_steer->data = 0.0;
    }
    publisher_steer->publish(*msg_steer);
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
    RCLCPP_INFO(this->get_logger(), "再稼働");
}

void ChassisDriver::_subscriber_callback_rpm(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
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
    // const double left_vel = (1.0 / wheel_radius) * linear_vel + (tread / wheel_radius*2.0) * angular_vel;
    // const double right_vel = (1.0 / wheel_radius) * linear_vel - (tread / wheel_radius*2.0) * angular_vel;
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
    int_to_bytes(_candata, static_cast<int>(right_rpm));
    int_to_bytes(_candata+4, static_cast<int>(left_rpm));

    for(int i=0; i<msg_can->candlc; i++) msg_can->candata[i]=_candata[i];
    publisher_can->publish(*msg_can);

}


}  // namespace chassis_driver
