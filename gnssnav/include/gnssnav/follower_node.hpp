#pragma once

#include "gnssnav/visibility_control.h"

#include <proj.h>

#include <cmath>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

namespace gnssnav{

class Follower : public rclcpp::Node{
public:
    Follower();

    void loop(void);
    int freq_;

    GNSSNAV_PUBLIC
    explicit Follower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    GNSSNAV_PUBLIC
    explicit Follower(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    bool autonomous_flag_;
    bool nav_start_flag_;
    bool init_base_flag_;
    bool init_flag_;

    double ld_;
    double ld_x_;
    double ld_y_;
    double base_x;
    double base_y;
    double base_z;
    double base_w;
    double ld_min_;
    double ld_gain_;
    double v_;
    double w_;
    double v_max_;
    double w_max_;
    double cte_gain_;
    double wheel_base_;
    double distance_;
    double theta;

    int freq;

    // geometry_msgs::msg::PoseStamped pose_msg;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vectornav_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr nav_start_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_ld_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> front_wheel_pos;
    std::vector<geometry_msgs::msg::PoseStamped> point_;

    void declareParameter(void);

    void vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void navStartCallback(const std_msgs::msg::Empty::SharedPtr&);
    void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
 
    double calculateCrossError();
    double calculateHeadingError();

    void findNearestIndex(const geometry_msgs::msg::Pose front_wheel_pos);
    void publishCurrentPose(void);
    void publishLookahead(void);
    void followPath();
    void findLookaheadDistance();
    void setBasePose(void);
    void calcPathDirection(void);

    double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion&);
    std::pair<double, double> convertECEFtoUTM(double x, double y, double z);
    double radian2deg(double rad);

    double current_position_x_;
    double current_position_y_;
    double current_yaw_;
    size_t idx_;
    size_t pre_point_idx;
    double vectornav_base_x_;
    double vectornav_base_y_;
    double vectornav_base_yaw_;
    double path_direction_;
    double pose_orientation_z_;
    double count;
    double init_yaw;

    std::vector<geometry_msgs::msg::Point> pre_point;
};

}  // namespace gnssnav
