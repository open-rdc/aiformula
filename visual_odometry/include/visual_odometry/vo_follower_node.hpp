#pragma once

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "utilities/position_pid.hpp"
#include "utilities/utils.hpp"

#include <vector>
#include <cmath>

namespace vonav
{

class PurePursuitFollower : public rclcpp::Node
{
public:
    explicit PurePursuitFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    /* ================= CALLBACK ================= */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /* ================= CONTROL ================= */
    void controlLoop();

    /* ================= PURE PURSUIT ================= */
    void findLookaheadPoint();
    double calcHeadingError();

    /* ================= VISUALIZATION ================= */
    void publishLookahead();
    void publishCurrentPose();

    /* ================= UTILS ================= */
    double yawFromQuat(const geometry_msgs::msg::Quaternion& q);

private:
    /* ---------- ROS Interface ---------- */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* ---------- State ---------- */
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t target_idx_{0}, prev_idx_{0};

    double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
    double base_x_{0.0}, base_y_{0.0}, base_yaw_{0.0};

    bool base_initialized_{false};
    bool autonomous_{false};

    /* ---------- Parameters ---------- */
    int freq_ms_;
    double ld_gain_;
    double ld_min_;
    double v_max_;
    double w_max_;

    controller::PositionPid pid_;
};

} // namespace vonav
