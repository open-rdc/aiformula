#include "cybergear_interface/cybergear_interface_node.hpp"
#include "cybergear_interface/cybergear_defs.h"
#include "utilities/utils.hpp"

using namespace utils;

namespace cybergear_interface{

CybergearInterface::CybergearInterface(const rclcpp::NodeOptions& options) : CybergearInterface("", options) {}

CybergearInterface::CybergearInterface(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("cybergear_interface_node", name_space, options),
driver(get_parameter("master_id").as_int(), get_parameter("target_id").as_int(),
    this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("cybergear/can_tx", _qos)),
interval_ms(get_parameter("interval_ms").as_int()),
gear_rate(get_parameter("gear_rate").as_double()),
is_reversed(get_parameter("reverse_flag").as_bool()),

pos_limit_min(dtor(get_parameter("pos_limit_min").as_double()) * gear_rate),
pos_limit_max(dtor(get_parameter("pos_limit_max").as_double()) * gear_rate),
limit_speed(dtor(get_parameter("limit_speed").as_double()))
{
    _subscription_pos = this->create_subscription<std_msgs::msg::Float64>(
        "cybergear/pos",
        _qos,
        std::bind(&CybergearInterface::_subscriber_callback_pos, this, std::placeholders::_1)
    );
    _subscription_reset = this->create_subscription<std_msgs::msg::Empty>(
        "cybergear/reset",
        _qos,
        std::bind(&CybergearInterface::_subscriber_callback_reset, this, std::placeholders::_1)
    );
    _subscription_stop = this->create_subscription<std_msgs::msg::Empty>(
        "stop",
        _qos,
        std::bind(&CybergearInterface::_subscriber_callback_stop, this, std::placeholders::_1)
    );
    _subscription_restart = this->create_subscription<std_msgs::msg::Empty>(
        "restart",
        _qos,
        std::bind(&CybergearInterface::_subscriber_callback_restart, this, std::placeholders::_1)
    );

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    // driver.change_motor_boardrate(0x02);
    driver.init_motor(cybergear_defs::MODE::POSITION);
    driver.set_limit_speed(static_cast<float>(limit_speed*gear_rate));
    driver.enable_motor();

    RCLCPP_INFO(this->get_logger(), "init cybergear interface node");
    RCLCPP_INFO(this->get_logger(), "マスターID:0x%03X  ターゲットID:0x%03X", get_parameter("master_id").as_int(), get_parameter("target_id").as_int());
    RCLCPP_INFO(this->get_logger(), "逆転:%d  最大位置:%.3lf  最小位置:%.3lf", is_reversed, pos_limit_max, pos_limit_min);
}

void CybergearInterface::_publisher_callback(){
    // driver.set_speed_ref(1.5f);
    if(mode == Mode::stop || mode == Mode::stay){
        return;
    }

    if(driver.get_run_mode() == cybergear_defs::MODE::POSITION){
        double limit_min, limit_max;
        if(is_reversed){
            limit_min = -this->pos_limit_max;
            limit_max = -this->pos_limit_min;
        }
        else{
            limit_min = this->pos_limit_min;
            limit_max = this->pos_limit_max;
        }
        driver.set_position_ref(static_cast<float>(pos_ref), static_cast<float>(pos_limit_min), static_cast<float>(pos_limit_max));
    }
}

void CybergearInterface::_subscriber_callback_pos(const std_msgs::msg::Float64::SharedPtr msg){
    if(mode == Mode::stop) return;

    if(driver.get_run_mode() != cybergear_defs::MODE::POSITION) driver.init_motor(cybergear_defs::MODE::POSITION);
    this->pos_ref = (is_reversed ? -1 : 1) * msg->data * gear_rate;
    // RCLCPP_INFO(this->get_logger(), "目標位置を設定");
    mode = Mode::cmd;
}

void CybergearInterface::_subscriber_callback_reset(const std_msgs::msg::Empty::SharedPtr msg){
    driver.set_mech_position_to_zero();
    RCLCPP_INFO(this->get_logger(), "機構の零点を現在値に変更");
    mode = Mode::stay;
}
void CybergearInterface::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stop;
}
void CybergearInterface::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
    mode = Mode::stay;
}

}  // namespace cybergear_interface
