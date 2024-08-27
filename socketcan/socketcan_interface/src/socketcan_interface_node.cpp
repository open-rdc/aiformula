#include <rclcpp/rclcpp.hpp>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <cerrno>
#include <fcntl.h>
#include <cstdio>
#include <boost/format.hpp>
#include <fstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "socketcan_interface/socketcan_interface_node.hpp"

namespace socketcan_interface {

SocketcanInterface::SocketcanInterface(const rclcpp::NodeOptions &options) : SocketcanInterface("", options) {}

SocketcanInterface::SocketcanInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("socketcan_interface_node", name_space, options),
if_name(get_parameter("if_name").as_string()),
ignoreid_file_path(ament_index_cpp::get_package_share_directory("socketcan_interface")+"/config/"+"ignoreid.cfg")
{
    using namespace std::chrono_literals;

    declare_parameter("interval_ms", 1);
    auto interval_ms = this->get_parameter("interval_ms").as_int();

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        RCLCPP_ERROR(this->get_logger(), "Socket error");
    }

    const std::string name = "can" + if_name;
    strcpy(ifr.ifr_name, name.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("Bind");
        RCLCPP_ERROR(this->get_logger(), "Bind error");
    }

    if (fcntl(s, F_SETFL, O_NONBLOCK) < 0) {    // ノンブロッキングモード
        perror("Setting");
        RCLCPP_ERROR(this->get_logger(), "Can setting error");
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"init socket can interface node");

    // 無視するIDをベクターに入れる
    RCLCPP_INFO_STREAM(this->get_logger(),"以下のCAN IDを無視し、トピックには流しません。");
    ignoreid.reserve(20);
    using namespace std;
    ifstream ifs(ignoreid_file_path);
    string str;
    while(getline(ifs, str)){
        const int id = stoi(str, nullptr, 16);
        RCLCPP_INFO(this->get_logger(), "CAN ID:0x%03X", id);

        ignoreid.push_back(id);
    }

    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    _subscription = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_tx",
        _qos,
        std::bind(&SocketcanInterface::_subscriber_callback, this, std::placeholders::_1)
    );

}

void SocketcanInterface::_publisher_callback() {
    auto msg = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();

    int nbytes;
    struct can_frame frame{};
    errno = 0;

    while (true) {
        nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno !=
                (EAGAIN | EWOULDBLOCK)) {   // ノンブロッキングモードで何も読み取れない場合、エラーを出す
                RCLCPP_ERROR(this->get_logger(), "Unexpected read error.");
            }
            break;
        }
        // 無視するIDであれば戻らせる
        bool is_ignoreid = false;
        for (const auto& id : ignoreid) {
            if(frame.can_id == id){
                is_ignoreid = true;
                break;
            }
        }
        if(is_ignoreid)continue;

        msg->header.stamp = this->now();

        // 拡張CANIDフォーマットかどうか確認
        if(frame.can_id & CAN_EFF_FLAG){
            msg->canid = frame.can_id & CAN_EFF_MASK;
            msg->eff = true;
        }
        else{
            msg->canid = frame.can_id & CAN_SFF_MASK;
            msg->eff = false;
        }

        msg->candlc = frame.can_dlc;
        for (int i = 0; i < frame.can_dlc; i++) {
            msg->candata[i] = frame.data[i];
        }

        if (known_id_rx_publisher.find(frame.can_id) == known_id_rx_publisher.end()) {
            known_id_rx_publisher[frame.can_id] = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>(
                    std::string("can_rx_" + (boost::format("%x") % frame.can_id).str()), _qos);
        }
        known_id_rx_publisher[frame.can_id]->publish(*msg);
    }

}

void SocketcanInterface::_subscriber_callback(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    struct can_frame frame{};

    // 確認CANIDフォーマットかどうか
    if(msg->eff){
        frame.can_id = CAN_EFF_FLAG | (msg->canid & CAN_EFF_MASK);
    }
    else{
        frame.can_id = msg->canid & CAN_SFF_MASK;
    }

    if ((msg->candlc >= 0) and (msg->candlc <= 8)) {
        frame.can_dlc = msg->candlc;
    } else {
        frame.can_dlc = 0;
    }
    std::string can_data_print;
    // std::move(msg->candata.begin(), msg->candata.end(), std::begin(frame.data));
    for (int i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = msg->candata[i];

        char str[64];
        sprintf(str, "0x%03X, ", msg->candata[i]);
        can_data_print = can_data_print + str;
    }
    // RCLCPP_INFO(this->get_logger(), "Sending to can bus ID: 0x%X, can data: %s", msg->canid, can_data_print.c_str());

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write frame0");
        RCLCPP_ERROR(this->get_logger(), "Write error");
    }
}
}
