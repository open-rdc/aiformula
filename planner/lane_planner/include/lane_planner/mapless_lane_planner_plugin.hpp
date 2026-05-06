#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <lanelet2_core/primitives/Lanelet.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mapless_planning_msgs/msg/driving_corridor.hpp>
#include <mapless_planning_msgs/msg/mission_lanes_stamped.hpp>
#include <mapless_planning_msgs/msg/road_segments.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lane_planner/lane_planner_plugin.hpp"
#include "lane_planner/lanelet_utils.hpp"

namespace lane_planner
{

class MaplessLanePlannerPlugin : public LanePlannerPlugin
{
public:
    void initialize(rclcpp::Node & node) override;
    void set_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override;
    void set_nav_command(const std::string & command) override;
    void set_road_segments(const mapless_planning_msgs::msg::RoadSegments & segments) override;
    std::optional<nav_msgs::msg::Path> compute_path(const rclcpp::Time & stamp) override;
    std::optional<mapless_planning_msgs::msg::MissionLanesStamped>
    get_mission_lanes() const override;
    std::optional<visualization_msgs::msg::MarkerArray> get_markers() const override;

private:
    using Point = geometry_msgs::msg::Point;

    mapless_planning_msgs::msg::DrivingCorridor build_ego_corridor(
        const mapless_planning_msgs::msg::RoadSegments & segments);

    nav_msgs::msg::Path corridor_to_path(
        const mapless_planning_msgs::msg::DrivingCorridor & corridor,
        const geometry_msgs::msg::Pose & ego_in_map,
        const rclcpp::Time & stamp) const;

    static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

    // Parameters
    std::string map_frame_id_{"map"};
    double max_corridor_length_m_{20.0};
    double path_resample_interval_m_{0.5};
    int target_lane_{0};  // 0=LANE_KEEP, -1=LEFT, +1=RIGHT

    // Runtime state
    mutable std::mutex data_mutex_;
    std::optional<mapless_planning_msgs::msg::RoadSegments> latest_segments_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    std::optional<mapless_planning_msgs::msg::MissionLanesStamped> latest_mission_lanes_;
    std::vector<lanelet::Lanelet> latest_lanelets_;

    rclcpp::Logger logger_{rclcpp::get_logger("mapless_lane_planner_plugin")};
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace lane_planner
