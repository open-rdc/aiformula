#include "controller/controller_node.hpp"
#include "utilities/data_utils.hpp"

using namespace std;
using namespace utils;

namespace controller{

Controller::Controller(const rclcpp::NodeOptions& options) : Controller("", options) {}

Controller::Controller(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("controller_node", name_space, options),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max.vel").as_double())),
is_autonomous(get_parameter("autonomous_flag").as_bool())
{
    _subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        _qos,
        std::bind(&Controller::_subscriber_callback_joy, this, std::placeholders::_1)
    );

    publisher_vel = this->create_publisher<geometry_msgs::msg::Vector3>("cmd_vel", _qos);
    publisher_stop = this->create_publisher<std_msgs::msg::Empty>("stop", _qos);
    publisher_restart = this->create_publisher<std_msgs::msg::Empty>("restart", _qos);
    publisher_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency", _qos);
    publisher_autonomous = this->create_publisher<std_msgs::msg::Bool>("autonomous", _qos);
    publisher_nav_start = this->create_publisher<std_msgs::msg::Empty>("nav_start", _qos);

    // 駆動系に電源が行っている可能性もあるのでリスタートする
    publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
}

void Controller::_subscriber_callback_joy(const sensor_msgs::msg::Joy::SharedPtr msg){

    // 緊急停止(未実装だが今後のため)
    if(upedge_ps(msg->buttons[static_cast<int>(Buttons::PS)])){
        publisher_stop->publish(*std::make_shared<std_msgs::msg::Empty>());

        auto msg_emergency = std::make_shared<std_msgs::msg::Bool>();
        msg_emergency->data = is_emergency = !is_emergency;
        publisher_emergency->publish(*msg_emergency);
        RCLCPP_INFO(this->get_logger(), "緊急停止フラグ : %d", msg_emergency->data);
    }
    // 自律か手動か
    if(upedge_share(msg->buttons[static_cast<int>(Buttons::Share)])){
        auto msg_autonomous = std::make_shared<std_msgs::msg::Bool>();
        msg_autonomous->data = is_autonomous = !is_autonomous;
        publisher_autonomous->publish(*msg_autonomous);
        RCLCPP_INFO(this->get_logger(), "自動フラグ : %d", msg_autonomous->data);
    }
    // 自律走行開始
    if(upedge_share(msg->buttons[static_cast<int>(Buttons::Triangle)])){
        publisher_nav_start->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "自律走行開始");
    }
    // リスタート
    if(upedge_options(msg->buttons[static_cast<int>(Buttons::Options)])){
        publisher_restart->publish(*std::make_shared<std_msgs::msg::Empty>());
        RCLCPP_INFO(this->get_logger(), "再稼働");
    }
    // 手動の場合、速度指令値を送る
    if(!is_autonomous){
        auto msg_vel = std::make_shared<geometry_msgs::msg::Vector3>();
        msg_vel->x = linear_max_vel * msg->axes[static_cast<int>(Axes::L_y)];
        msg_vel->z = angular_max_vel * msg->axes[static_cast<int>(Axes::R_x)];
        publisher_vel->publish(*msg_vel);
    }

}

}  // namespace controller