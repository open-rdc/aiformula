#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vectormap_msgs/msg/lane_connection.hpp>
#include <vectormap_msgs/msg/lanelet.hpp>
#include <vectormap_msgs/msg/line_string.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "vectormap_global_planner/visibility_control.h"

namespace vectormap_global_planner
{

class VectormapGlobalPlannerNode : public rclcpp::Node
{
public:
    VECTORMAP_GLOBAL_PLANNER_PUBLIC
    explicit VectormapGlobalPlannerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VECTORMAP_GLOBAL_PLANNER_PUBLIC
    explicit VectormapGlobalPlannerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    struct Point2D
    {
        double x;
        double y;
    };

    struct PathPoint
    {
        double s;
        double x;
        double y;
        double yaw;
        uint64_t lanelet_id;
    };

    struct LaneletRange
    {
        uint64_t lanelet_id;
        double start_s;
        double end_s;
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
    void nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();

    void build_global_path_once(const vectormap_msgs::msg::VectorMap& map_msg);
    void build_route_from_lanelet_ids(const std::vector<uint64_t>& route_lanelet_ids);
    void rebuild_route_from_lanelet(uint64_t start_lanelet_id, const std::string& reason);
    void request_route_rebuild(const std::string& reason);
    bool rebuild_route_from_pose(const Point2D& ego, const std::string& reason);
    std::vector<uint64_t> build_route_sequence_from_graph(
        uint64_t start_lanelet_id,
        std::size_t& fallback_count) const;
    uint64_t select_next_lanelet(
        uint64_t from_lanelet_id,
        uint8_t requested_turn,
        uint8_t& selected_turn,
        bool& used_fallback) const;
    uint64_t find_nearest_lanelet_from_pose(const Point2D& point) const;
    uint64_t lanelet_at_s(double s) const;
    double normalize_path_s(double s) const;
    uint8_t parse_nav_cmd(const std::string& command) const;
    std::string turn_direction_to_string(uint8_t turn_direction) const;
    nav_msgs::msg::Path make_global_path_message(const rclcpp::Time& stamp) const;
    static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

    const int update_period_ms_;
    const std::string map_frame_id_;
    const std::string base_frame_id_;
    const std::string vector_map_topic_;
    const std::string localization_pose_topic_;
    const std::string nav_cmd_topic_;
    const std::string default_nav_cmd_;
    const std::string global_path_topic_;
    const std::vector<int64_t> route_lanelet_ids_param_;
    const std::vector<std::string> nav_cmd_fallback_order_param_;
    const double global_path_resample_interval_m_;
    const double max_centerline_connection_gap_m_;
    const int route_lookahead_lanelet_count_;
    const rclcpp::QoS qos_;

    bool global_path_ready_;
    bool route_is_loop_;
    bool pending_route_rebuild_;
    std::string pending_route_rebuild_reason_;
    uint64_t route_start_lanelet_id_;
    uint8_t last_nav_cmd_turn_;
    std::vector<uint8_t> nav_cmd_fallback_order_;

    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;
    std::vector<LaneletRange> lanelet_ranges_;
    std::unordered_map<uint64_t, vectormap_msgs::msg::Lanelet> lanelet_by_id_;
    std::unordered_map<uint64_t, std::vector<Point2D>> lanelet_centerline_points_by_id_;
    std::unordered_map<uint64_t, std::vector<RouteEdge>> connection_edges_by_from_lanelet_id_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_cmd_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vectormap_global_planner
