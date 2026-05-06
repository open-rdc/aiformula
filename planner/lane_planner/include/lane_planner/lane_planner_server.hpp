#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mapless_planning_msgs/msg/mission_lanes_stamped.hpp>
#include <mapless_planning_msgs/msg/road_segments.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lane_planner/lane_planner_plugin.hpp"
#include "lane_planner/visibility_control.h"

namespace lane_planner
{

class LanePlannerServer : public rclcpp::Node
{
public:
    LANE_PLANNER_PUBLIC
    explicit LanePlannerServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    LANE_PLANNER_PUBLIC
    explicit LanePlannerServer(
        const std::string & name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg);
    void road_segments_callback(
        const mapless_planning_msgs::msg::RoadSegments::SharedPtr msg);
    void timer_callback();

    const int update_period_ms_;
    const rclcpp::QoS qos_;

    std::unique_ptr<LanePlannerPlugin> plugin_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<mapless_planning_msgs::msg::RoadSegments>::SharedPtr road_segments_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<mapless_planning_msgs::msg::MissionLanesStamped>::SharedPtr mission_lanes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lane_planner
