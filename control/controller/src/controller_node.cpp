#include "controller/controller_node.hpp"
#include "utilities/data_utils.hpp"

using namespace std;
using namespace utils;

namespace controller{

Controller::Controller(const rclcpp::NodeOptions& options) : Controller("", options) {}

Controller::Controller(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("controller_node", name_space, options),
linear_max_vel(get_parameter("linear_max_vel").as_double()),
steering_max_angle(dtor(get_parameter("steering_max.pos").as_double()))
{
    _subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        _qos,
        std::bind(&Controller::_subscriber_callback_joy, this, std::placeholders::_1)
    );

    publisher_vel = this->create_publisher<steered_drive_msg::msg::SteeredDrive>("cmd_vel", _qos);
    publisher_restart = this->create_publisher<std_msgs::msg::Empty>("restart", _qos);
    publisher_autonomous = this->create_publisher<std_msgs::msg::Bool>("autonomous", _qos);
    publisher_nav_cmd = this->create_publisher<std_msgs::msg::String>("/planning/nav_cmd", _qos);
    publisher_lane_switch_flag = this->create_publisher<std_msgs::msg::Empty>("/flag", _qos);

    // 駆動系に電源が行っている可能性もあるのでリスタートする
    // publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
}

void Controller::_subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg){
    // 自動か手動か
    if(upedge_share(msg->buttons[static_cast<int>(Buttons::Share)])){
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous = !is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), "自動フラグ : %d", msg_autonomous->data);
    }
    // リスタート
    if(upedge_options(msg->buttons[static_cast<int>(Buttons::Options)])){
        publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "再稼働");
    }
    // 経路選択
    const bool l1_button = msg->buttons[static_cast<int>(Buttons::L1)];
    const bool r1_button = msg->buttons[static_cast<int>(Buttons::R1)];
    const bool l2_button = msg->buttons[static_cast<int>(Buttons::L2)];
    const bool l1_pressed = upedge_l1(l1_button);
    const bool r1_pressed = upedge_r1(r1_button);
    const bool l2_pressed = upedge_l2(l2_button);
    const bool l1_released = downedge_l1(l1_button);
    const bool r1_released = downedge_r1(r1_button);
    const bool l2_released = downedge_l2(l2_button);

    if(l1_pressed || l2_pressed){
        publish_nav_cmd("left");
    }
    if(r1_pressed){
        publish_nav_cmd("right");
    }
    if((l1_released || r1_released || l2_released) && !l1_button && !r1_button && !l2_button){
        publish_nav_cmd("straight");
    }
    // レーン切り替え
    if(upedge_cross(msg->buttons[static_cast<int>(Buttons::Cross)])){
        publisher_lane_switch_flag->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "レーン切り替え");
    }
    // 手動の場合、速度指令値を送る
    if(!is_autonomous){
        auto msg_vel = std::make_shared<steered_drive_msg::msg::SteeredDrive>();
        msg_vel->velocity = linear_max_vel * msg->axes[static_cast<int>(Axes::L_y)];
        msg_vel->steering_angle = steering_max_angle * msg->axes[static_cast<int>(Axes::R_x)];
        publisher_vel->publish(*msg_vel);
    }

}

void Controller::publish_nav_cmd(const std::string& command){
    auto msg_nav_cmd = std::make_shared<std_msgs::msg::String>();
    msg_nav_cmd->data = command;
    publisher_nav_cmd->publish(*msg_nav_cmd);
    RCLCPP_INFO(this->get_logger(), "経路選択 : %s", msg_nav_cmd->data.c_str());
}

}  // namespace controller
