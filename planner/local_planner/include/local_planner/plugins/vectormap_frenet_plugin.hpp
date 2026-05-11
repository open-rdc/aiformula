#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

#include "local_planner/local_planner_plugin.hpp"

namespace local_planner
{

class VectormapFrenetPlugin : public LocalPlannerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    void setGlobalPath(const nav_msgs::msg::Path & global_path) override;

    void setVectorMap(const vectormap_msgs::msg::VectorMap & map) override;

    void requestLaneChange() override;

    std::optional<nav_msgs::msg::Path> computeLocalPath(
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects) override;

private:
    struct Point2D { double x; double y; };
    struct PathPoint { double s; double x; double y; double yaw; uint64_t lanelet_id; };
    struct FrenetPoint { double s; double d; };
    struct FrenetObstacle { double s; double d; };

    enum class LaneChangeState { Idle, Executing };

    void start_lane_switch(double current_s, const Point2D& ego);
    void start_lane_offset_transition(double current_s, double target_offset, const std::string& reason);

    std::vector<PathPoint> generate_local_path(
        const FrenetPoint & ego_frenet,
        bool has_obstacle,
        const FrenetObstacle & obstacle,
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
        const FrenetObstacle & obstacle,
        double avoidance_shift,
        double avoidance_start_s,
        double avoidance_end_s,
        double avoidance_return_start_s,
        double avoidance_return_end_s) const;
    bool is_collision_free(
        const std::vector<PathPoint> & candidate,
        const FrenetObstacle & obstacle) const;
    double evaluate_frenet_candidate(
        const std::vector<PathPoint> & candidate,
        double target_offset,
        double avoidance_shift) const;
    bool find_static_obstacle(
        double current_s,
        double base_offset,
        const object_detection_msgs::msg::ObjectInfoArray & objects,
        double & obstacle_s,
        double & obstacle_d,
        double & avoidance_shift) const;

    FrenetPoint project_to_path(const Point2D & point) const;
    PathPoint path_point_at_s(double s) const;
    double max_path_s() const;
    double normalize_path_s(double s) const;
    uint64_t find_nearest_lanelet(const Point2D & point) const;
    double adjacent_lane_offset(uint64_t lanelet_id, bool to_left) const;

    static double smooth_step(double t);
    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
    static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
    static double point_segment_distance_sq(
        const Point2D & point, const Point2D & start, const Point2D & end);

    nav_msgs::msg::Path make_path_message(
        const std::vector<PathPoint> & points,
        const rclcpp::Time & stamp) const;

    rclcpp::Logger logger_{rclcpp::get_logger("vectormap_frenet_plugin")};
    rclcpp::Clock::SharedPtr clock_;

    double local_path_horizon_m_{15.0};
    double local_path_resample_interval_m_{0.2};
    double max_centerline_connection_gap_m_{0.5};
    double vehicle_width_m_{0.6};
    double lane_change_length_m_{10.0};
    double avoidance_detection_forward_distance_m_{15.0};
    double avoidance_hard_margin_m_{0.2};
    double avoidance_soft_margin_m_{0.3};
    double envelope_buffer_margin_m_{0.2};
    double avoidance_lateral_jerk_mps3_{1.0};
    double avoidance_min_velocity_mps_{0.5};
    double max_avoidance_shift_m_{1.0};
    double frenet_collision_check_margin_m_{0.2};
    double frenet_weight_lateral_offset_{1.0};
    double frenet_weight_lateral_change_{0.2};
    double frenet_weight_avoidance_shift_{0.1};
    std::string map_frame_id_{"map"};
    std::string base_frame_id_{"base_link"};

    bool global_path_ready_{false};
    bool route_is_loop_{false};
    bool lane_change_requested_{false};
    bool lane_switch_completed_{false};

    LaneChangeState lane_change_state_{LaneChangeState::Idle};
    double active_lane_offset_m_{0.0};
    double lane_change_start_s_{0.0};
    double lane_change_end_s_{0.0};
    double lane_change_start_offset_m_{0.0};
    double lane_change_target_offset_m_{0.0};

    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;

    std::unordered_map<uint64_t, std::vector<Point2D>> lanelet_centerline_points_by_id_;
    std::unordered_map<uint64_t, uint64_t> left_adjacent_lanelet_by_id_;
    std::unordered_map<uint64_t, uint64_t> right_adjacent_lanelet_by_id_;
};

}  // namespace local_planner
