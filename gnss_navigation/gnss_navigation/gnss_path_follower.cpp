#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower"), current_goal_index_(0) {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/gnss_path", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
        followPath();
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
        current_goal_index_ = 0;  // パスが更新されたときに目標インデックスをリセット
    }

    void followPath() {
        if (path_.empty() || current_goal_index_ >= path_.size()) return;

        auto& goal_pose = path_[current_goal_index_].pose.position;
        double dx = goal_pose.x - current_position_x_;
        double dy = goal_pose.y - current_position_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        geometry_msgs::msg::Twist cmd_vel;
        if (distance > 0.1) {  // 目標に十分近づくまで移動
            cmd_vel.linear.x = std::min(1.0, 0.1 * distance);  // 簡単なプロポーショナル制御
            cmd_vel.angular.z = std::atan2(dy, dx);  // 方向を目標に合わせる
        } else {
            current_goal_index_++;  // 次の目標に移動
        }
        cmd_pub_->publish(cmd_vel);
    }

    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
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
    return 0;
}

