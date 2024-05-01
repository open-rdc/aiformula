#include "debug_printer/debug_printer.hpp"
#include "utilities/data_utils.hpp"

namespace debug_printer{
DebugPrinter::DebugPrinter(const rclcpp::NodeOptions &options) : DebugPrinter("", options) {}

DebugPrinter::DebugPrinter(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("debug_printer_node", name_space, options){

    _subscription_rpm_rx = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
            "can_rx_110",
            _qos,
            std::bind(&DebugPrinter::_subscriber_callback_rpm_rx, this, std::placeholders::_1)
    );
    _subscription_can_tx = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
            "can_tx",
            _qos,
            std::bind(&DebugPrinter::_subscriber_callback_can_tx, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ロボテックモータドライバーに関するデータを整形してトピックに流します。");
}

void DebugPrinter::_subscriber_callback_rpm_rx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    auto msg_tx = std::make_shared<std_msgs::msg::Float64>();

    publisher_float64 = this->create_publisher<std_msgs::msg::Float64>("left_rpm_rx", _qos);
    const double left = msg_tx->data = static_cast<double>(bytes_to_float(_candata));
    publisher_float64->publish(*msg_tx);

    publisher_float64 = this->create_publisher<std_msgs::msg::Float64>("right_rpm_rx", _qos);
    const double right = msg_tx->data = static_cast<double>(bytes_to_float(_candata+4));
    publisher_float64->publish(*msg_tx);

    RCLCPP_INFO(this->get_logger(), "RPM TX   L:%lf  R:%lf", left, right);
}
void DebugPrinter::_subscriber_callback_can_tx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    if(msg->canid == 0x210 && msg->candlc == 8){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

        auto msg_tx = std::make_shared<std_msgs::msg::Float64>();

        publisher_float64 = this->create_publisher<std_msgs::msg::Float64>("left_rpm_tx", _qos);
        const double left = msg_tx->data = static_cast<double>(bytes_to_float(_candata));
        publisher_float64->publish(*msg_tx);

        publisher_float64 = this->create_publisher<std_msgs::msg::Float64>("right_rpm_tx", _qos);
        const double right = msg_tx->data = static_cast<double>(bytes_to_float(_candata+4));
        publisher_float64->publish(*msg_tx);

        RCLCPP_INFO(this->get_logger(), "RPM RX   L:%lf  R:%lf", left, right);
    }
}


}   // namespace

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<debug_printer::DebugPrinter>());
  rclcpp::shutdown();
  return 0;
}
