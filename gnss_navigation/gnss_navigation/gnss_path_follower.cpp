#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2 geometry_msgs extensions
#include <proj.h>


class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode()
    : Node("path_follower"),
      autonomous_flag_(false),
      nav_start_flag_(false),
      vectornav_init_flag_(true),
      lookahead_distance_(1.5),  // 先読み距離
      max_linear_velocity_(1.0),
      max_angular_velocity_(0.7),
      goal_tolerance_(0.5) {
        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/vectornav/pose", 10, std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));
        auto callback = [this](const std_msgs::msg::Empty::SharedPtr msg) { this->nav_start_Callback(msg); };
        nav_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/nav_start", 10, callback);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/origin_gnss_path", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cmd_vel", 10);
        to_target_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/to_target", 10);
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
        flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/autonomous", 10, std::bind(&PathFollowerNode::flagCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Need ECEF->WGS84->UTM function
        auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        current_position_x_ = x;
        current_position_y_ = y;
        current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation);
        // current_yaw_ = std::atan2(std::sin(current_yaw_diff), std::cos(current_yaw_diff));
        RCLCPP_INFO(this->get_logger(), "pose_x:%f, pose_y:%f, pose_yaw:%f", current_position_x_, current_position_y_, current_yaw_);
        publish_current_pose();
        followPath();
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
    }

    void nav_start_Callback(const std_msgs::msg::Empty::SharedPtr&) {
        nav_start_flag_ = true;
        RCLCPP_INFO(this->get_logger(), "自律走行開始");
    }

    void flagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        autonomous_flag_ = msg->data;  // autonomous_flag_を更新
        RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
    }

    void publish_current_pose()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now(); 
        pose_msg.header.frame_id = "map"; 

        pose_msg.pose.position.x = current_position_x_;  
        pose_msg.pose.position.y = current_position_y_;  
        pose_msg.pose.position.z = 0.0;  

        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);  // Roll, Pitch, Yawからクォータニオンを計算
        pose_msg.pose.orientation = tf2::toMsg(q);
        current_pose_pub_->publish(pose_msg);
    }

    void followPath() {
        if (path_.empty()) return;

        if (!nav_start_flag_) return;

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
        // double accumulated_distance = 0.0;
        size_t lookahead_index = target_index;
        for (size_t i = target_index; i < path_.size() - 1; ++i) {
            double segment_length = std::hypot(
                path_[i + 1].pose.position.x - current_position_x_,
                path_[i + 1].pose.position.y - current_position_y_);
            // accumulated_distance += segment_length;
            if (segment_length >= lookahead_distance_) {
                lookahead_index = i + 1;
                break;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "%d", lookahead_index);

        auto& lookahead_pose = path_[lookahead_index].pose.position;
        double dx = lookahead_pose.x - current_position_x_;
        double dy = lookahead_pose.y - current_position_y_;
        double distance_to_lookahead = std::sqrt(dx * dx + dy * dy);

        double target_angle = std::atan2(dy, dx);
        double angle_difference = target_angle - current_yaw_;
        angle_difference = std::atan2(std::sin(angle_difference), std::cos(angle_difference));

        // double curvature = 2 * std::sin(angle_difference) / distance_to_lookahead;

        double controlled_angular_speed = std::copysign(std::min(std::abs(angle_difference), max_angular_velocity_), angle_difference);

        // double controlled_linear_speed = std::max(std::min(max_linear_velocity_, max_linear_velocity_ - std::abs(controlled_angular_speed) * k_vel), 0.2);

        double controlled_linear_speed = std::min(max_linear_velocity_, distance_to_lookahead);

        if (lookahead_index >= path_.size() - 10)
        {
            controlled_linear_speed = std::min(max_linear_velocity_, distance_to_lookahead * 0.6);
        }

        geometry_msgs::msg::Vector3 to_target;
        to_target.x = distance_to_lookahead;
        to_target.y = controlled_linear_speed;
        to_target.z = controlled_angular_speed;

        to_target_pub_->publish(to_target);

        RCLCPP_INFO(this->get_logger(), "Current distance to target: %f meters, Current angle to target: %f radians", distance_to_lookahead, target_angle);

        geometry_msgs::msg::Vector3 cmd_vel;
        cmd_vel.x = controlled_linear_speed;
        cmd_vel.z = controlled_angular_speed;

        if ((distance_to_lookahead < 0.1) && (lookahead_index >= path_.size() - 10)){
            cmd_vel.x = 0.0;
            cmd_vel.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reached goal");
        }

        if (autonomous_flag_ == true)
        {
            cmd_pub_->publish(cmd_vel);
        }
    }

    double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    std::pair<double, double> convertECEFtoUTM(double x, double y, double z) {
        PJ_CONTEXT *C = proj_context_create();
        PJ *P = proj_create_crs_to_crs(C,
                                   "EPSG:4978", // ECEF
                                   "EPSG:32654", // UTMゾーン54N
                                   NULL);
        if (P == NULL) {
        std::cerr << "PROJ transformation creation failed." << std::endl;
        }
        PJ_COORD a, b;

        a.xyz.x = x;
        a.xyz.y = y;
        a.xyz.z = z;

        b = proj_trans(P, PJ_FWD, a);

        // リソースの解放
        proj_destroy(P);
        proj_context_destroy(C);

        return {b.enu.e, b.enu.n};
    }

    bool autonomous_flag_;
    bool nav_start_flag_;
    bool vectornav_init_flag_;
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_yaw_diff = 0.0;
    double vectornav_base_x = 0.0;
    double vectornav_base_y = 0.0;
    double vectornav_base_yaw = 0.0;
    double k_vel = 3;
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr nav_start_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr to_target_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
