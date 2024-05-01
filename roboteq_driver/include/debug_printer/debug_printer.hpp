#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace debug_printer{
class DebugPrinter : public rclcpp::Node{
public:
    explicit DebugPrinter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit DebugPrinter(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_rpm_rx;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_can_tx;

    void _subscriber_callback_rpm_rx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_can_tx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_float64;

    rclcpp::QoS _qos = rclcpp::QoS(10);
};
}
