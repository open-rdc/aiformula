#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_velocity_body_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_markers_;

    rclcpp::QoS qos_{
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
        rmw_qos_profile_sensor_data
    };


    nav_msgs::msg::Path::SharedPtr global_path_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_;
    nav_msgs::msg::Path::SharedPtr local_path_;
    geometry_msgs::msg::Twist current_twist_;

    RiskCalculator risk_calculator_;
    FrenetPlanner frenet_planner_;
    ObstacleDetector obstacle_detector_;

    const double wheelbase_;
    const double caster_max_angle_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void velocity_body_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
};

}  // namespace frenet_planner
