#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2 geometry_msgs extensions

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
    : Node("path_follower"),
      autonomous_flag_(false), // Autonomous flag 初期値は false
      k_p_linear_(1.0),  // 線形速度の比例定数
      lookahead_distance_(3.0),  // 先読み距離
      max_linear_velocity_(3.0),
      max_angular_velocity_(2.0),
      goal_tolerance_(0.5) {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/gnss_path", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cmd_vel", 10);
        flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/autonomous_flag", 10, std::bind(&PathFollowerNode::flagCallback, this, std::placeholders::_1));
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
    }

    void flagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        autonomous_flag_ = msg->data;  // autonomous_flag_を更新
        RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
    }

    void followPath() {
        if (path_.empty() || !autonomous_flag_) {
            geometry_msgs::msg::Vector3 cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_pub_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "Autonomous mode is off or path is empty. Stopping the robot.");
            return;
        }

        // 先読み点を探索
        double closest_distance = std::numeric_limits<double>::max();
        size_t target_index = 0;
        for (size_t i = 0; i < path_.size(); ++i) {
            double dx = path_[i].pose.position.x - current_position_x_;
            double dy = path_[i].pose.position.y - current_position_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < closest_distance) {
                closest_distance = distance;
                target_index = i;
            }
        }

        // 先読み点を見つける
        double accumulated_distance = 0.0;
        size_t lookahead_index = target_index;
        for (size_t i = target_index; i < path_.size() - 1; ++i) {
            double segment_length = std::hypot(
                path_[i + 1].pose.position.x - path_[i].pose.position.x,
                path_[i + 1].pose.position.y - path_[i].pose.position.y);
            accumulated_distance += segment_length;
            if (accumulated_distance >= lookahead_distance_) {
                lookahead_index = i + 1;
                break;
            }
        }

        auto& lookahead_pose = path_[lookahead_index].pose.position;
        double dx = lookahead_pose.x - current_position_x_;
        double dy = lookahead_pose.y - current_position_y_;
        double distance_to_lookahead = std::sqrt(dx * dx + dy * dy);

        double target_angle = std::atan2(dy, dx);
        double angle_difference = target_angle - current_yaw_;
        angle_difference = std::atan2(std::sin(angle_difference), std::cos(angle_difference));

        RCLCPP_INFO(this->get_logger(), "Current distance to target: %f meters, Current angle to target: %f radians", distance_to_lookahead, angle_difference);

        geometry_msgs::msg::Twist cmd_vel;
            RCLCPP_INFO(this->get_logger(), "Autonomous mode is off. Stopping the robot.");
            double controlled_linear_speed = std::min(max_linear_velocity_, k_p_linear_ * distance_to_lookahead);
            double controlled_angular_speed = std::copysign(std::min(std::abs(angle_difference), max_angular_velocity_), angle_difference);

            cmd_vel.linear.x = controlled_linear_speed;
            cmd_vel.angular.z = controlled_angular_speed;

        if(autonomous_flag_ == true)
        cmd_pub_->publish(cmd_vel);
    }

    double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool autonomous_flag_;
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_yaw_ = 0.0;
    double k_p_linear_;
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
