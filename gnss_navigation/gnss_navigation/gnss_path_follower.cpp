#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower"), current_goal_index_(0) {
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
        current_velocity_x_ = msg->twist.twist.linear.x;
        current_velocity_y_ = msg->twist.twist.linear.y;
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
        double target_angle = std::atan2(dy, dx);

        // 現在の速度ベクトルの角度を計算
        double velocity_angle = std::atan2(current_velocity_y_, current_velocity_x_);

        // 目標への角度と現在の進行角度の差
        double angle_difference = target_angle - velocity_angle;
        angle_difference = std::atan2(std::sin(angle_difference), std::cos(angle_difference));

        geometry_msgs::msg::Twist cmd_vel;
        double distance = std::sqrt(dx * dx + dy * dy);

        if(distance > 1.0){
        	cmd_vel.linear.x = std::min(20.0, (0.5 * distance) + 10);
        	cmd_vel.angular.z = 8.0 * angle_difference;
        } else {
        	current_goal_index_++;  // 次の目標に移動
        }

        cmd_pub_->publish(cmd_vel);
    }

    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_velocity_x_ = 0.0;
    double current_velocity_y_ = 0.0;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_goal_index_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
