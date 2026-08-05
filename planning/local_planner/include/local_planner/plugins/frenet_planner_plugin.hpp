#pragma once

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "local_planner/frenet/soft_constraint.hpp"
#include "local_planner/frenet/structures.hpp"
#include "local_planner/local_planner_plugin.hpp"

namespace local_planner
{

class FrenetPlannerPlugin : public LocalPlannerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    void setGlobalPath(const nav_msgs::msg::Path & global_path) override;

    std::optional<nav_msgs::msg::Path> computeLocalPath(
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects) override;

private:
    struct Point2D { double x; double y; };
    struct PathPoint { double s; double x; double y; double yaw; };
    struct CartesianPoint { double x; double y; double yaw; };
    struct ProjectedPose { double s; double d; double path_yaw; };
    struct FrenetObstacle { double s; double d; double half_width; };

    std::vector<CartesianPoint> plan_best_path(
        double start_s,
        double end_s,
        const frenet::FrenetState & initial,
        const std::optional<FrenetObstacle> & obstacle) const;
    std::vector<CartesianPoint> make_stop_path(
        const frenet::FrenetState & initial,
        const FrenetObstacle & obstacle) const;
    std::vector<double> make_target_s_list(double start_s, double end_s) const;
    std::vector<double> make_target_grid(const std::optional<FrenetObstacle> & obstacle) const;
    std::optional<FrenetObstacle> find_static_obstacle(
        double current_s,
        const object_detection_msgs::msg::ObjectInfoArray & objects) const;

    std::vector<PathPoint> sample_reference(const std::vector<double> & s_grid) const;
    std::vector<CartesianPoint> to_cartesian(
        const std::vector<PathPoint> & reference,
        const std::vector<double> & offsets) const;

    ProjectedPose project_to_path(const Point2D & point) const;
    PathPoint path_point_at_s(double s) const;
    double max_path_s() const;
    double normalize_path_s(double s) const;
    double reference_curvature_at(double s) const;
    static std::vector<double> compute_curvatures(const std::vector<CartesianPoint> & points);
    static double compute_path_length(const std::vector<CartesianPoint> & points);

    nav_msgs::msg::Path make_path_message(
        const std::vector<CartesianPoint> & points,
        const rclcpp::Time & stamp) const;

    rclcpp::Logger logger_{rclcpp::get_logger("frenet_planner_plugin")};
    rclcpp::Clock::SharedPtr clock_;

    double local_path_horizon_m_{15.0};
    double local_path_resample_interval_m_{0.2};
    double max_centerline_connection_gap_m_{0.5};
    double vehicle_width_m_{0.6};
    double avoidance_detection_forward_distance_m_{15.0};
    double avoidance_hard_margin_m_{0.2};
    double avoidance_soft_margin_m_{0.3};
    double envelope_buffer_margin_m_{0.2};
    double max_avoidance_shift_m_{1.0};
    double frenet_lateral_sample_step_m_{0.25};
    double frenet_collision_check_margin_m_{0.2};
    std::vector<double> frenet_target_lengths_m_{7.5, 15.0};
    frenet::CostWeights cost_weights_{2000.0, 1.0, 50.0};
    double stop_standoff_m_{1.0};
    double kappa_max_{0.0};

    bool global_path_ready_{false};
    bool route_is_loop_{false};

    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;
};

}
