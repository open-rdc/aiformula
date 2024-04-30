#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2 geometry_msgs extensions

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
    : Node("path_follower"),
      k_p_linear_(1.0), // 線形速度の比例定数
      k_p_angular_(2.0), // 角速度の比例定数
      current_goal_index_(0),
      max_linear_velocity_(3.0),
      max_angular_velocity_(2.0),
      goal_tolerance_(0.5) {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/gnss_path", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
        current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation);
        followPath();
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
        current_goal_index_ = 0;
    }

    void followPath() {
        if (path_.empty() || current_goal_index_ >= path_.size()) return;

        auto& goal_pose = path_[current_goal_index_].pose.position;
        double dx = goal_pose.x - current_position_x_;
        double dy = goal_pose.y - current_position_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_difference = target_angle - current_yaw_;
        angle_difference = std::atan2(std::sin(angle_difference), std::cos(angle_difference));

        RCLCPP_INFO(this->get_logger(), "Current distance to target: %f meters, Current angle to target: %f radians", distance, target_angle);

        double controlled_linear_speed = std::min(max_linear_velocity_, k_p_linear_ * distance);
        double controlled_angular_speed = std::min(max_angular_velocity_, k_p_angular_ * std::abs(angle_difference)) * std::copysign(1.0, angle_difference);

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = controlled_linear_speed;
        cmd_vel.angular.z = controlled_angular_speed;

        cmd_pub_->publish(cmd_vel);

        if (distance < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached, moving to next goal");
            current_goal_index_++;
        }
    }

    double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_yaw_ = 0.0;  // Added missing semicolon here
    double k_p_linear_;
    double k_p_angular_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    size_t current_goal_index_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

