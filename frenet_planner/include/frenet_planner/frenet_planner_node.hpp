#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rmw/qos_profiles.h>

#include "frenet_planner/visibility_control.h"
#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/obstacle_detect.hpp"

namespace frenet_planner {

class FrenetPlannerNode : public rclcpp::Node {
public:
    FRENET_PLANNER_PUBLIC
    explicit FrenetPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    FRENET_PLANNER_PUBLIC
    explicit FrenetPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_autonomous_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_markers_;

    rclcpp::QoS qos_{
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
        rmw_qos_profile_sensor_data
    };

    bool autonomous_flag_ = false;

    nav_msgs::msg::Path::SharedPtr global_path_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_;
    nav_msgs::msg::Path::SharedPtr local_path_;
    geometry_msgs::msg::Twist current_twist_;

    std::unique_ptr<FrenetPlanner> frenet_planner_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    std::shared_ptr<RiskCalculator> risk_calculator_;

    const double wheelbase_;
    const double caster_max_angle_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace frenet_planner
