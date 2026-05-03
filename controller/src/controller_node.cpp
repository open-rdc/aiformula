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

    // 駆動系に電源が行っている可能性もあるのでリスタートする
    // publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
}

void Controller::_subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg){
    const auto share_idx = static_cast<int>(Buttons::Share);
    const auto options_idx = static_cast<int>(Buttons::Options);
    const auto ps_idx = static_cast<int>(Buttons::PS);

    // 自動か手動か
    if(share_idx < static_cast<int>(msg->buttons.size()) && upedge_share(msg->buttons[share_idx])){
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous = !is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), "自動フラグ : %d", msg_autonomous->data);
    }
    // PSボタン(buttons[10])で自動走行モードへ移行し、Pure Pursuitを有効化
    if(ps_idx < static_cast<int>(msg->buttons.size()) && upedge_ps(msg->buttons[ps_idx])){
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous = !is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), is_autonomous ? "自動走行モードへ切替: Pure Pursuit 有効" : "手動走行モードへ切替");
    }
    // リスタート
    if(options_idx < static_cast<int>(msg->buttons.size()) && upedge_options(msg->buttons[options_idx])){
        publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "再稼働");
    }
    // 手動の場合、速度指令値を送る
    if(!is_autonomous){
        auto msg_vel = std::make_shared<steered_drive_msg::msg::SteeredDrive>();
        msg_vel->velocity = linear_max_vel * msg->axes[static_cast<int>(Axes::L_y)];
        msg_vel->steering_angle = steering_max_angle * msg->axes[static_cast<int>(Axes::R_x)];
        publisher_vel->publish(*msg_vel);
    }

}

}  // namespace controller
