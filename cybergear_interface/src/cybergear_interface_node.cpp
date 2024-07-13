#include "cybergear_interface/cybergear_interface_node.hpp"
#include "cybergear_interface/cybergear_defs.h"

namespace cybergear_interface{

CybergearInterface::CybergearInterface(const rclcpp::NodeOptions& options) : CybergearInterface("", options) {}

CybergearInterface::CybergearInterface(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("cybergear_interface_node", name_space, options),
driver(get_parameter("master_id").as_int(), get_parameter("target_id").as_int(),
    this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos)),
interval_ms(get_parameter("interval_ms").as_int())
{
    _subscription_pos = this->create_subscription<geometry_msgs::msg::Vector3>(
        "pos",
        _qos,
        std::bind(&CybergearInterface::_subscriber_callback_pos, this, std::placeholders::_1)
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

    RCLCPP_INFO(this->get_logger(), "target_id : 0x%03X", get_parameter("target_id").as_int());

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    driver.init_motor(cybergear_defs::MODE::SPEED);

    init_speed = 10.0;
    driver.set_limit_speed(static_cast<float>(init_speed));
    driver.enable_motor();
}

void CybergearInterface::_publisher_callback(){
    // driver.motor_control(6.28f, 3.14f, 0.0f, 1.0f, 1.0f);
    driver.set_speed_ref(1.5f);
}

void CybergearInterface::_subscriber_callback_pos(const geometry_msgs::msg::Vector3::SharedPtr msg){
}
void CybergearInterface::_subscriber_callback_stop(const std_msgs::msg::Empty::SharedPtr msg){
}
void CybergearInterface::_subscriber_callback_restart(const std_msgs::msg::Empty::SharedPtr msg){
}

}  // namespace cybergear_interface
