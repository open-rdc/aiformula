#include "debug_printer/debug_printer.hpp"
#include "utilities/data_utils.hpp"

namespace debug_printer{
DebugPrinter::DebugPrinter(const rclcpp::NodeOptions &options) : DebugPrinter("", options) {}

DebugPrinter::DebugPrinter(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("debug_printer_node", name_space, options){

    _subscription_rpm_rx = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
            "can_rx_711",
            _qos,
            std::bind(&DebugPrinter::_subscriber_callback_rpm_rx, this, std::placeholders::_1)
    );
    _subscription_potentio = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
            "can_rx_11",
            _qos,
            std::bind(&DebugPrinter::_subscriber_callback_potentio, this, std::placeholders::_1)
    );
    _subscription_can_tx = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
            "can_tx",
            _qos,
            std::bind(&DebugPrinter::_subscriber_callback_can_tx, this, std::placeholders::_1)
    );

    publisher_left_rpm_rx = this->create_publisher<std_msgs::msg::Int64>("left_rpm_rx", _qos);
    publisher_right_rpm_rx = this->create_publisher<std_msgs::msg::Int64>("right_rpm_rx", _qos);
    publisher_left_rpm_tx = this->create_publisher<std_msgs::msg::Int64>("left_rpm_tx", _qos);
    publisher_right_rpm_tx = this->create_publisher<std_msgs::msg::Int64>("right_rpm_tx", _qos);
    publisher_potentio = this->create_publisher<std_msgs::msg::Int64>("potentio", _qos);



    RCLCPP_INFO(this->get_logger(), "ロボテックモータドライバーに関するデータを整形してトピックに流します。");
}

void DebugPrinter::_subscriber_callback_rpm_rx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    auto msg_tx = std::make_shared<std_msgs::msg::Int64>();

    const int left = msg_tx->data = static_cast<int64_t>(bytes_to_int(_candata));
    publisher_left_rpm_rx->publish(*msg_tx);

    const int right = msg_tx->data = static_cast<int64_t>(bytes_to_int(_candata+4));
    publisher_right_rpm_rx->publish(*msg_tx);

    RCLCPP_DEBUG(this->get_logger(), "RPM RX   L:%d  R:%d", left, right);
}
void DebugPrinter::_subscriber_callback_can_tx(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    if(msg->canid == 0x210 && msg->candlc == 8){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

        auto msg_tx = std::make_shared<std_msgs::msg::Int64>();

        const int left = msg_tx->data = static_cast<int64_t>(bytes_to_int(_candata));
        publisher_left_rpm_tx->publish(*msg_tx);

        const int right = msg_tx->data = static_cast<int64_t>(bytes_to_int(_candata+4));
        publisher_right_rpm_tx->publish(*msg_tx);

        RCLCPP_DEBUG(this->get_logger(), "RPM TX   L:%d  R:%d", left, right);
    }
}

void DebugPrinter::_subscriber_callback_potentio(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];

    auto msg_tx = std::make_shared<std_msgs::msg::Int64>();
    const int value = msg_tx->data = static_cast<int64_t>(bytes_to_short(_candata));
    publisher_potentio->publish(*msg_tx);
    RCLCPP_DEBUG(this->get_logger(), "POTENTIO:%d", value);
}


}   // namespace

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<debug_printer::DebugPrinter>());
  rclcpp::shutdown();
  return 0;
}
