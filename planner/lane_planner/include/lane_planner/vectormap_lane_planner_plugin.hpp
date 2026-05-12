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
#include <vectormap_msgs/msg/lane_connection.hpp>
#include <vectormap_msgs/msg/lanelet.hpp>
#include <vectormap_msgs/msg/line_string.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "lane_planner/lane_planner_plugin.hpp"

namespace lane_planner
{

class VectormapLanePlannerPlugin : public LanePlannerPlugin
{
public:
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

    void initialize(rclcpp::Node & node) override;
    void set_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override;
    void set_nav_command(const std::string & command) override;
    std::optional<nav_msgs::msg::Path> compute_path(const rclcpp::Time & stamp) override;

private:
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);

    void build_global_path_once(const vectormap_msgs::msg::VectorMap & map_msg);
    void build_route_from_lanelet_ids(const std::vector<uint64_t> & route_lanelet_ids);
    void rebuild_route_from_lanelet(uint64_t start_lanelet_id, const std::string & reason);
    bool rebuild_route_from_pose(const Point2D & ego, const std::string & reason);
    std::vector<uint64_t> build_route_sequence_from_graph(
        uint64_t start_lanelet_id, std::size_t & fallback_count) const;
    uint64_t select_next_lanelet(
        uint64_t from_lanelet_id, uint8_t requested_turn,
        uint8_t & selected_turn, bool & used_fallback) const;
    uint64_t find_nearest_lanelet_from_pose(const Point2D & point) const;
    std::pair<uint64_t, double> find_nearest_lanelet_within_route(const Point2D & point) const;
    uint64_t lanelet_at_s(double s) const;
    double normalize_path_s(double s) const;
    uint8_t parse_nav_cmd(const std::string & command) const;
    std::string turn_direction_to_string(uint8_t turn_direction) const;
    nav_msgs::msg::Path make_path_message(const rclcpp::Time & stamp) const;
    static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

    // Parameters (read during initialize())
    std::string map_frame_id_;
    std::string vector_map_topic_;
    std::string default_nav_cmd_;
    std::vector<int64_t> route_lanelet_ids_param_;
    std::vector<std::string> nav_cmd_fallback_order_param_;
    double global_path_resample_interval_m_{0.5};
    double max_centerline_connection_gap_m_{0.5};
    double off_route_distance_threshold_m_{3.0};
    int route_lookahead_lanelet_count_{10};

    // Runtime state
    bool global_path_ready_{false};
    bool route_is_loop_{false};
    bool pending_route_rebuild_{false};
    std::string pending_route_rebuild_reason_;
    std::vector<uint64_t> current_route_lanelet_ids_;
    uint8_t last_nav_cmd_turn_{0};
    std::vector<uint8_t> nav_cmd_fallback_order_;
    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;
    std::vector<LaneletRange> lanelet_ranges_;
    std::unordered_map<uint64_t, vectormap_msgs::msg::Lanelet> lanelet_by_id_;
    std::unordered_map<uint64_t, std::vector<Point2D>> lanelet_centerline_points_by_id_;
    std::unordered_map<uint64_t, std::vector<RouteEdge>> connection_edges_by_from_lanelet_id_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_sub_;
    rclcpp::Logger logger_{rclcpp::get_logger("vectormap_lane_planner_plugin")};
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace lane_planner
