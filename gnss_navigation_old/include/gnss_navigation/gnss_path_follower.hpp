#ifndef GNSS_PATH_FOLLOWER_HPP
#define GNSS_PATH_FOLLOWER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <proj.h>

namespace gnss_navigation
{

class GNSSPathFollower : public rclcpp::Node
{
    public:
        GNSSPathFollower();
        ~GNSSPathFollower();

        void loop(void);
        int freq_;

    private:
        bool autonomous_flag_;
        bool nav_start_flag_;
        bool init_base_flag_;

        double lookahead_distance_;
        double max_linear_vel_;
        double max_angular_vel_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vectornav_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr nav_start_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_flag_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_lookahead_pub_;

        std::vector<geometry_msgs::msg::PoseStamped> path_;

        void declareParameter(void);
        void initCommunication(void);

        void vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void navStartCallback(const std_msgs::msg::Empty::SharedPtr&);
        void autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

        void publishCurrentPose(void);
        void publishLookahead(void);
        void followPath(void);
        void findLookaheadDistance(void);
        void setBasePose(void);
        void calcPathDirection(void);

        double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion&);
        std::pair<double, double> convertECEFtoUTM(double x, double y, double z);
        double radian2deg(double rad);

        double current_position_x_;
        double current_position_y_;
        double current_yaw_;
        size_t lookahead_index_;
        double vectornav_base_x_;
        double vectornav_base_y_;
        double vectornav_base_yaw_;
        double path_direction_;
};

} // namespace gnss_navigation

#endif // GNSS_PATH_FOLLOWER_HPP