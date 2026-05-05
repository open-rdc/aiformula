#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mapless_planning_msgs/msg/mission_lanes_stamped.hpp>
#include <mapless_planning_msgs/msg/road_segments.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lane_planner
{

class LanePlannerPlugin
{
public:
    virtual ~LanePlannerPlugin() = default;

    virtual void initialize(rclcpp::Node & node) = 0;

    virtual void set_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped & pose) = 0;

    virtual void set_nav_command(const std::string & command) = 0;

    virtual std::optional<nav_msgs::msg::Path> compute_path(
        const rclcpp::Time & stamp) = 0;

    // Optional setter — only used by mapless plugin; default is no-op
    virtual void set_road_segments(
        const mapless_planning_msgs::msg::RoadSegments & /*segments*/) {}

    // Optional — only used by mapless plugin; default returns nullopt
    virtual std::optional<mapless_planning_msgs::msg::MissionLanesStamped>
    get_mission_lanes() const { return std::nullopt; }
};

}  // namespace lane_planner
