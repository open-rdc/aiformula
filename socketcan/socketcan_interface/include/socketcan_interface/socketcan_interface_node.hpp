#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <net/if.h>
#include <sys/socket.h>

#include <linux/can.h>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "visibility.h"

namespace socketcan_interface {

class SocketcanInterface : public rclcpp::Node {
public:
    SOCKETCAN_INTERFACE_PUBLIC
    explicit SocketcanInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SOCKETCAN_INTERFACE_PUBLIC
    explicit SocketcanInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription;
    rclcpp::TimerBase::SharedPtr _pub_timer;
    std::map<uint16_t, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr> known_id_rx_publisher;

    void _publisher_callback();
    void _subscriber_callback(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    rclcpp::QoS _qos = rclcpp::QoS(40);

    // CAN
    struct sockaddr_can addr{};
    struct ifreq ifr{};
    int s;
    const std::string if_name; // CANデバイスの識別
    const std::string ignoreid_file_path;   //無視するCAN IDリストのファイルパス
    std::vector<int> ignoreid;

};
}
