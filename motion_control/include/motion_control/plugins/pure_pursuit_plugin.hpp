#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "motion_control/controller_plugin.hpp"

namespace motion_control
{

// Frenet sampling + Pure Pursuit を統合した local planner プラグイン．
// global path を入力とし，frenet 空間で候補軌道を生成・評価したうえで
// Pure Pursuit による速度・舵角指令を出力する．
class PurePursuitPlugin : public ControllerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseWithCovarianceStamped * ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped * velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects,
        geometry_msgs::msg::PoseStamped & target_pose_out) override;

private:
    struct Point2D { double x; double y; };
    struct PathPoint { double s; double x; double y; double yaw; };
    struct FrenetPoint { double s; double d; };
    struct FrenetObstacle { double s; double d; };

    // Global path をスプライン補間に必要な s/x/y/yaw 列に変換し保持する．
    // size と先頭座標から変化を検出して再構築する．
    void update_global_samples(const nav_msgs::msg::Path & path);

    std::vector<PathPoint> generate_local_path(
        const FrenetPoint & ego_frenet,
        bool has_obstacle,
        const FrenetObstacle & obstacle,
        double avoidance_shift,
        double speed_mps) const;

    std::vector<PathPoint> sample_frenet_path(
        double start_s,
        double end_s,
        double start_d,
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
        const object_detection_msgs::msg::ObjectInfoArray & objects,
        double & obstacle_s,
        double & obstacle_d,
        double & avoidance_shift) const;

    FrenetPoint project_to_path(const Point2D & point) const;
    PathPoint path_point_at_s(double s) const;
    double max_path_s() const;
    double normalize_path_s(double s) const;
    static double smooth_step(double t);
    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);

    // Pure Pursuit 部
    struct TargetPoint { double x; double y; };
    static std::vector<PathPoint> transform_local_to_base(
        const std::vector<PathPoint> & local,
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose);
    bool find_lookahead_target(
        const std::vector<PathPoint> & path_base,
        TargetPoint & target_out) const;

    rclcpp::Logger logger_{rclcpp::get_logger("pure_pursuit_plugin")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>()};

    // Pure pursuit パラメータ
    double linear_max_vel_{0.0};
    double lookahead_distance_{0.0};
    double steered_gain_{0.0};
    double wheelbase_{0.0};
    double steering_max_angle_rad_{0.0};

    // Frenet sampling パラメータ
    double local_path_horizon_m_{15.0};
    double local_path_resample_interval_m_{0.2};
    double max_centerline_connection_gap_m_{0.5};
    double vehicle_width_m_{0.6};
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

    // Global path 状態
    bool global_path_ready_{false};
    bool route_is_loop_{false};
    std::size_t last_path_size_{0U};
    double last_first_x_{0.0};
    double last_first_y_{0.0};
    std::string path_frame_id_;
    std::vector<PathPoint> global_samples_;
};

}  // namespace motion_control
