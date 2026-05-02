#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vectormap_msgs/msg/lane_connection.hpp>
#include <vectormap_msgs/msg/lanelet.hpp>
#include <vectormap_msgs/msg/line_string.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "vectormap_planner/visibility_control.h"

namespace vectormap_planner
{

class VectormapPlannerNode : public rclcpp::Node
{
public:
    VECTORMAP_PLANNER_PUBLIC
    explicit VectormapPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VECTORMAP_PLANNER_PUBLIC
    explicit VectormapPlannerNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

    struct FrenetPoint
    {
        double s;
        double d;
    };

    struct FrenetObstacle
    {
        double s;
        double d;
    };

    enum class LaneChangeState
    {
        Idle,
        Executing,
    };

private:
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void change_lane_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void timer_callback();

    void build_global_path_once(const vectormap_msgs::msg::VectorMap& map_msg);
    void build_adjacency(
        const std::unordered_map<uint64_t, vectormap_msgs::msg::Lanelet>& lanelet_by_id);
    void start_lane_change(double current_s, uint64_t current_lanelet_id);

    nav_msgs::msg::Path make_path_message(
        const std::vector<PathPoint>& points,
        const rclcpp::Time& stamp) const;
    std::vector<PathPoint> generate_local_path(
        const FrenetPoint& ego_frenet,
        bool has_obstacle,
        const FrenetObstacle& obstacle,
        double avoidance_shift,
        double speed_mps);
    std::vector<PathPoint> sample_frenet_path(
        double start_s,
        double end_s,
        double start_d,
        double lane_change_start_s,
        double lane_change_end_s,
        double lane_change_start_offset,
        double lane_change_end_offset,
        double fallback_target_offset,
        const FrenetObstacle& obstacle,
        double avoidance_shift,
        double avoidance_start_s,
        double avoidance_end_s,
        double avoidance_return_start_s,
        double avoidance_return_end_s) const;
    bool is_collision_free(
        const std::vector<PathPoint>& candidate,
        const FrenetObstacle& obstacle) const;
    double evaluate_frenet_candidate(
        const std::vector<PathPoint>& candidate,
        double target_offset,
        double avoidance_shift) const;

    FrenetPoint project_to_path(const Point2D& point) const;
    PathPoint path_point_at_s(double s) const;
    double max_path_s() const;
    double normalize_path_s(double s) const;
    uint64_t lanelet_at_s(double s) const;
    double adjacent_lane_offset(uint64_t lanelet_id, bool to_left) const;
    bool find_static_obstacle(
        double current_s,
        double base_offset,
        const geometry_msgs::msg::PoseWithCovarianceStamped& current_pose,
        const sensor_msgs::msg::PointCloud2& pointcloud,
        double& obstacle_s,
        double& obstacle_d,
        double& avoidance_shift);

    static double smooth_step(double t);
    static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

    const int update_period_ms_;
    const std::string map_frame_id_;
    const std::string base_frame_id_;
    const std::string lane_change_direction_;
    const std::string vector_map_topic_;
    const std::string localization_pose_topic_;
    const std::string velocity_topic_;
    const std::string pointcloud_topic_;
    const std::string change_lane_topic_;
    const std::string global_path_topic_;
    const std::string local_path_topic_;
    const std::vector<int64_t> route_lanelet_ids_param_;
    const double global_path_resample_interval_m_;
    const double local_path_horizon_m_;
    const double local_path_resample_interval_m_;
    const double max_centerline_connection_gap_m_;
    const double vehicle_width_m_;
    const double lane_change_length_m_;
    const double avoidance_detection_forward_distance_m_;
    const double avoidance_hard_margin_m_;
    const double avoidance_soft_margin_m_;
    const double envelope_buffer_margin_m_;
    const double avoidance_lateral_jerk_mps3_;
    const double avoidance_min_velocity_mps_;
    const double max_avoidance_shift_m_;
    const double frenet_collision_check_margin_m_;
    const double frenet_weight_lateral_offset_;
    const double frenet_weight_lateral_change_;
    const double frenet_weight_avoidance_shift_;
    const int obstacle_pointcloud_step_;
    const rclcpp::QoS qos_;

    bool global_path_ready_;
    bool route_is_loop_;
    bool previous_change_lane_signal_;
    LaneChangeState lane_change_state_;
    double active_lane_offset_m_;
    double lane_change_start_s_;
    double lane_change_end_s_;
    double lane_change_start_offset_m_;
    double lane_change_target_offset_m_;

    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;
    std::vector<LaneletRange> lanelet_ranges_;
    std::unordered_map<uint64_t, std::vector<Point2D>> lanelet_centerline_points_by_id_;
    std::unordered_map<uint64_t, uint64_t> left_adjacent_lanelet_by_id_;
    std::unordered_map<uint64_t, uint64_t> right_adjacent_lanelet_by_id_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr latest_velocity_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    mutable std::mutex data_mutex_;

    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr change_lane_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vectormap_planner
