#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "mission_planner/visibility_control.h"

namespace mission_planner
{

class MissionPlannerNode : public rclcpp::Node
{
public:
    MISSION_PLANNER_PUBLIC
    explicit MissionPlannerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    MISSION_PLANNER_PUBLIC
    explicit MissionPlannerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    struct Point2D
    {
        double x;
        double y;
    };

    struct PathPoint
    {
        double x;
        double y;
        double yaw;
    };

    struct RouteEdge
    {
        uint64_t to_lanelet_id;
        uint8_t turn_direction;
        double cost;
    };

private:
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void navigation_command_callback(const std_msgs::msg::String::SharedPtr msg);
    void lane_change_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void timer_callback();

    void build_map_lookup(const vectormap_msgs::msg::VectorMap& map_msg);
    bool try_start_initial_route(const Point2D& ego, double yaw);
    bool apply_route_lanelet_ids(const std::vector<uint64_t>& route_lanelet_ids);
    void replan_route_from_lanelet(uint64_t start_lanelet_id, const std::string& reason);
    bool replan_route_from_pose(const Point2D& ego, double yaw, const std::string& reason);
    std::vector<uint64_t> search_route_lanelet_ids(
        uint64_t start_lanelet_id,
        std::size_t& fallback_count) const;
    std::unordered_set<uint64_t> build_reachable_lanelet_set() const;
    std::pair<uint64_t, double> find_nearest_lanelet_within_route(const Point2D& point) const;
    nav_msgs::msg::Path make_global_path_message(const rclcpp::Time& stamp) const;
    void report_curvature_qa() const;

    const int update_period_ms_;
    const double global_path_resample_interval_m_;
    const double max_centerline_connection_gap_m_;
    const double off_route_distance_threshold_m_;
    const int route_lookahead_lanelet_count_;
    const double start_lanelet_yaw_threshold_rad_;
    const double start_lanelet_max_distance_m_;
    const double start_pose_position_variance_threshold_;
    const double route_extension_min_remaining_m_;
    const double curvature_limit_per_m_;
    const std::vector<uint8_t> navigation_command_fallback_order_;

    bool map_ready_;
    bool global_path_ready_;
    bool current_route_is_loop_;
    uint8_t last_navigation_command_turn_;
    std::vector<uint64_t> current_route_lanelet_ids_;

    std::vector<PathPoint> global_samples_;
    std::unordered_map<uint64_t, std::vector<Point2D>> lanelet_centerline_points_by_id_;
    std::unordered_map<uint64_t, std::vector<RouteEdge>> connection_edges_by_from_lanelet_id_;
    std::unordered_map<uint64_t, uint64_t> left_adjacent_lanelet_by_id_;
    std::unordered_map<uint64_t, uint64_t> right_adjacent_lanelet_by_id_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_command_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr lane_change_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
