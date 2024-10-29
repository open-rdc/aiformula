#pragma once

#include "gnssnav/visibility_control.h"

#include <proj.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utilities/position_pid.hpp"

namespace gnssnav{

class Follower : public rclcpp::Node{
public:
    GNSSNAV_PUBLIC
    explicit Follower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    GNSSNAV_PUBLIC
    explicit Follower(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    GNSSNAV_PUBLIC ~Follower();

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vectornav_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr nav_start_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr restart_subscriber_;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_ld_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> point_;

    void vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void navStartCallback(const std_msgs::msg::Empty::SharedPtr&);
    void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void restartCallback(const std_msgs::msg::Empty::SharedPtr msg);

    void publishCurrentPose(void);
    void publishLookahead(void);
    void followPath();
    double findLookaheadDistance();
    void setBasePose(void);
    double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion&);
    double calculateCrossError();

    std::pair<double, double> convertECEFtoUTM(double x, double y, double z);

    controller::PositionPid pid;

    const int freq_ms;
    const int is_debug;
    const double ld_min_;
    const double ld_gain_;
    const double v_max_;
    const double w_max_;
    const double wheel_base_;
    int laps;


    bool autonomous_flag_=false;
    bool nav_start_flag_=false;
    bool init_base_flag_=false;
    bool init_d=false;

    double v_=0;
    double w_=0;

    double theta_error=0;
    double theta_sum=0;
    double prev_theta=0;
    double current_position_x_=0;
    double current_position_y_=0;
    double current_yaw_=0;
    size_t idx_=0;
    size_t pre_point_idx=0;
    double vectornav_base_x_=0;
    double vectornav_base_y_=0;
    double vectornav_base_yaw_=0;

    PJ_CONTEXT *C;
    PJ *P;
};

}  // namespace gnssnav
