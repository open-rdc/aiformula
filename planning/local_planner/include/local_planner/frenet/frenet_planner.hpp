#pragma once

#include <vector>

#include "local_planner/frenet/reference_path.hpp"

namespace local_planner::frenet
{

struct Parameters
{
    double horizon_m;
    double resample_interval_m;
    double tread_m;
    double detection_forward_distance_m;
    double hard_margin_m;
    double soft_margin_m;
    double cluster_gap_m;
    double max_shift_m;
    double lateral_sample_step_m;
    double collision_check_margin_m;
    std::vector<double> avoidance_target_lengths_m;
    double merge_time_s;
    double merge_length_min_m;
    double merge_length_max_m;
    double merge_lateral_accel_mps2;
    double weight_curvature;
    double weight_lateral_deviation;
    double weight_side_change;
    double stop_standoff_m;
    double kappa_max;
};

struct Obstacle { double x; double y; double width; };
struct FrenetState { double s; double d; double d_prime; };  // d_prime は s に対する横位置の傾き
struct FrenetObstacle { double s; double d; double half_width; double half_length; };

FrenetState make_initial_state(const ReferencePath & reference, const Pose2D & ego, const Parameters & params);

// 前方・横範囲内の障害物を Frenet 座標へ変換し、隣接するものを 1 つにまとめる
std::vector<FrenetObstacle> find_obstacles(
    const ReferencePath & reference, double current_s, const std::vector<Obstacle> & obstacles,
    const Parameters & params);

// 参照線へ合流し終える縦位置の候補
std::vector<double> make_target_s_list(
    const FrenetState & initial, double end_s, double speed,
    const std::vector<FrenetObstacle> & obstacles, const Parameters & params);

// 合流後の横位置の候補。障害物から離れる側を先に並べる
std::vector<double> make_target_d_list(const std::vector<FrenetObstacle> & obstacles, const Parameters & params);

// (target_s, target_d) へ quintic で合流する候補のうち、衝突・曲率の制約を満たしコスト最小のものを返す
std::vector<Pose2D> select_best_path(
    const ReferencePath & reference, const FrenetState & initial, double end_s,
    const std::vector<double> & target_s_list, const std::vector<double> & target_d_list,
    const std::vector<FrenetObstacle> & obstacles, const std::vector<Pose2D> & previous_points,
    const Parameters & params);

}
