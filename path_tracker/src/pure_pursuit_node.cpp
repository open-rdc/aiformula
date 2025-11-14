#include "path_tracker/pure_pursuit_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

using namespace utils;

namespace path_tracker{

PurePursuit::PurePursuit(const rclcpp::NodeOptions& options) : PurePursuit("", options) {}

PurePursuit::PurePursuit(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pure_pursuit_node", name_space, options),
linear_max_vel(get_parameter("linear_max.vel").as_double()),
angular_max_vel(get_parameter("angular_max.vel").as_double()),
lookahead_distance(get_parameter("lookahead_distance").as_double())
{
    _subscription_path = this->create_subscription<nav_msgs::msg::Path>(
        "e2e_planner/path",
        _qos,
        std::bind(&PurePursuit::_subscriber_callback_path, this, std::placeholders::_1)
    );
    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

    RCLCPP_INFO(this->get_logger(), "PurePursuit node has been initialized. lookahead_distance: %.2f", lookahead_distance);
}

void PurePursuit::_subscriber_callback_path(const nav_msgs::msg::Path::SharedPtr msg){
    geometry_msgs::msg::Twist command;

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
    const double curvature = (20.0 * target_y) / (distance * distance);  // Pure pursuit curvature.
    double angular_velocity = linear_velocity * curvature;

    angular_velocity = std::clamp(angular_velocity, -angular_max_vel, angular_max_vel);

    command.linear.x = linear_velocity;
    command.angular.z = angular_velocity;
    publisher_vel->publish(command);
}


}  // namespace path_tracker
