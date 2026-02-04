#include "path_tracker/pure_pursuit_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

using namespace utils;

namespace path_tracker{

PurePursuit::PurePursuit(const rclcpp::NodeOptions& options) : PurePursuit("", options) {}

PurePursuit::PurePursuit(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pure_pursuit_node", name_space, options),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
lookahead_distance(get_parameter("lookahead_distance").as_double()),
wheelbase_(get_parameter("wheelbase").as_double()),
caster_max_angle_rad_(get_parameter("steering_max.pos").as_double() * 0.017453292519943295)
{
    _subscription_path = this->create_subscription<nav_msgs::msg::Path>(
        "/frenet_planner/path",
        _qos,
        std::bind(&PurePursuit::_subscriber_callback_path, this, std::placeholders::_1)
    );
    _subscription_autonomous = this->create_subscription<std_msgs::msg::Bool>(
        "/autonomous",
        _qos,
        std::bind(&PurePursuit::autonomous_callback, this, std::placeholders::_1)
    );
    publisher_vel = this->create_publisher<steered_drive_msg::msg::SteeredDrive>("cmd_vel", _qos);

    RCLCPP_INFO(this->get_logger(), "PurePursuit node has been initialized. lookahead_distance: %.2f", lookahead_distance);
}

void PurePursuit::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg){
    autonomous_flag_ = msg->data;
}

void PurePursuit::_subscriber_callback_path(const nav_msgs::msg::Path::SharedPtr msg){
    if (!autonomous_flag_) {
        return;
    }

    steered_drive_msg::msg::SteeredDrive command;

    if (!msg || msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Received empty path. Publishing zero velocity."
        );
        publisher_vel->publish(command);
        return;
    }

    // Select a lookahead pose measured in the robot base frame.
    auto target_it = std::find_if(
        msg->poses.begin(),
        msg->poses.end(),
        [this](const geometry_msgs::msg::PoseStamped& pose) {
            const double dx = pose.pose.position.x;
            const double dy = pose.pose.position.y;
            return std::hypot(dx, dy) >= lookahead_distance;
        }
    );

    if (target_it == msg->poses.end()) {
        target_it = std::prev(msg->poses.end());
    }

    const double target_x = target_it->pose.position.x;
    const double target_y = target_it->pose.position.y;
    const double distance = std::hypot(target_x, target_y);

    if (distance < 1e-6) {
        publisher_vel->publish(command);
        return;
    }

    const double safe_lookahead = std::max(lookahead_distance, 1e-3);
    const double linear_scale = std::clamp(distance / safe_lookahead, 0.0, 1.0);
    const double linear_velocity = std::clamp(linear_max_vel * linear_scale, 0.0, linear_max_vel);
    
    const double alpha = std::atan2(target_y, target_x);
    const double steer_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);
    const double steer_angle_clamped = std::clamp(steer_angle, -caster_max_angle_rad_, caster_max_angle_rad_);

    command.velocity = linear_velocity;
    command.steering_angle = steer_angle_clamped;
    publisher_vel->publish(command);
}


}  // namespace path_tracker
