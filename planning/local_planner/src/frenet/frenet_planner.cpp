#include "local_planner/frenet/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner::frenet
{
namespace
{
constexpr double EPSILON = 1.0e-6;
constexpr double COLLISION_LONGITUDINAL_MARGIN_M = 0.5;
constexpr double MAX_HEADING_ERROR_RAD = 0.25 * M_PI;
// quintic で横に d 動かすときの横方向 2 階微分の最大値は QUINTIC_PEAK_CURVATURE_FACTOR * d / L^2
constexpr double QUINTIC_PEAK_CURVATURE_FACTOR = 5.7735;

// target_s で横位置 target_d に合流する quintic を interval 間隔で標本化する
void sample_quintic(
    const FrenetState & initial, const double target_s, const double target_d, const double interval,
    std::vector<double> & s_points, std::vector<double> & offsets)
{
    const double T = target_s - initial.s;
    if (T < interval) {
        return;
    }
    const double dp = target_d - initial.d - initial.d_prime * T;
    const double dv = -initial.d_prime;
    const double c3 = (10.0 * dp - 4.0 * dv * T) / (T * T * T);
    const double c4 = (-15.0 * dp + 7.0 * dv * T) / (T * T * T * T);
    const double c5 = (6.0 * dp - 3.0 * dv * T) / (T * T * T * T * T);
    for (double s = interval; s <= T + EPSILON; s += interval) {
        s_points.push_back(initial.s + s);
        offsets.push_back(initial.d + s * (initial.d_prime + s * s * (c3 + s * (c4 + s * c5))));
    }
}

// 各点から前回経路までの距離の平均
double mean_deviation(const std::vector<Pose2D> & points, const std::vector<Pose2D> & previous)
{
    if (previous.size() < 2U) {
        return 0.0;
    }
    const auto distance_sq = [&previous](const Pose2D & point, const std::size_t i) {
        const auto & a = previous[i - 1U];
        const auto & b = previous[i];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double t = std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        const double ex = point.x - (a.x + t * dx);
        const double ey = point.y - (a.y + t * dy);
        return ex * ex + ey * ey;
    };

    double sum = 0.0;
    std::size_t i = 1U;
    for (const auto & point : points) {
        double nearest = distance_sq(point, i);
        while (i + 1U < previous.size()) {
            const double next = distance_sq(point, i + 1U);
            if (next >= nearest) {
                break;
            }
            nearest = next;
            ++i;
        }
        sum += std::sqrt(nearest);
    }
    return sum / static_cast<double>(points.size());
}
}

FrenetState make_initial_state(const ReferencePath & reference, const Pose2D & ego, const Parameters & params)
{
    const FrenetPose pose = reference.project(Point2D{ego.x, ego.y});
    const double heading_error = std::clamp(
        std::remainder(ego.yaw - pose.yaw, 2.0 * M_PI), -MAX_HEADING_ERROR_RAD, MAX_HEADING_ERROR_RAD);
    const double reference_curvature = reference.curvature_at(pose.s, params.resample_interval_m);
    return FrenetState{pose.s, pose.d, (1.0 - reference_curvature * pose.d) * std::tan(heading_error)};
}

std::vector<FrenetObstacle> find_obstacles(
    const ReferencePath & reference, const double current_s, const std::vector<Obstacle> & obstacles,
    const Parameters & params)
{
    const double lateral_limit = params.tread_m * 0.5 + params.hard_margin_m + params.soft_margin_m;

    std::vector<FrenetObstacle> found;
    for (const auto & obstacle : obstacles) {
        const FrenetPose pose = reference.project(Point2D{obstacle.x, obstacle.y});
        const double half_width = 0.5 * obstacle.width;
        const double delta_s = pose.s - current_s;
        if (delta_s < 0.0 || delta_s > params.detection_forward_distance_m ||
            std::abs(pose.d) > lateral_limit + half_width) {
            continue;
        }
        found.push_back(FrenetObstacle{pose.s, pose.d, half_width, 0.0});
    }
    std::sort(found.begin(), found.end(), [](const FrenetObstacle & a, const FrenetObstacle & b) { return a.s < b.s; });

    // 縦横とも cluster_gap_m 以内で隣接する障害物は 1 つにまとめる
    std::vector<FrenetObstacle> clusters;
    for (const auto & obstacle : found) {
        const double left = obstacle.d + obstacle.half_width;
        const double right = obstacle.d - obstacle.half_width;
        if (clusters.empty() ||
            obstacle.s - (clusters.back().s + clusters.back().half_length) > params.cluster_gap_m ||
            right - (clusters.back().d + clusters.back().half_width) > params.cluster_gap_m ||
            (clusters.back().d - clusters.back().half_width) - left > params.cluster_gap_m) {
            clusters.push_back(obstacle);
            continue;
        }
        auto & merged = clusters.back();
        const double s_front = std::min(merged.s - merged.half_length, obstacle.s);
        const double s_back = std::max(merged.s + merged.half_length, obstacle.s);
        const double d_left = std::max(merged.d + merged.half_width, left);
        const double d_right = std::min(merged.d - merged.half_width, right);
        merged = FrenetObstacle{
            0.5 * (s_front + s_back), 0.5 * (d_left + d_right), 0.5 * (d_left - d_right), 0.5 * (s_back - s_front)};
    }
    return clusters;
}

std::vector<double> make_target_s_list(
    const FrenetState & initial, const double end_s, const double speed,
    const std::vector<FrenetObstacle> & obstacles, const Parameters & params)
{
    // 障害物がなければ、車速に比例した距離（merge_time_s 秒分）で参照線へ合流する。
    if (obstacles.empty()) {
        const double accel_length =
            speed * std::sqrt(QUINTIC_PEAK_CURVATURE_FACTOR * std::abs(initial.d) / params.merge_lateral_accel_mps2);
        const double length = std::clamp(
            std::max(params.merge_time_s * speed, accel_length), params.merge_length_min_m, params.merge_length_max_m);
        return {std::min(end_s, initial.s + length)};
    }

    std::vector<double> targets;
    const double obstacle_s = obstacles.front().s - obstacles.front().half_length;
    if (obstacle_s > initial.s + params.resample_interval_m * 2.0 && obstacle_s <= end_s) {
        targets.push_back(obstacle_s);
    }
    for (const double length : params.avoidance_target_lengths_m) {
        targets.push_back(std::min(end_s, initial.s + length));
    }
    return targets;
}

std::vector<double> make_target_d_list(const std::vector<FrenetObstacle> & obstacles, const Parameters & params)
{
    const int step_count = static_cast<int>(std::ceil(params.max_shift_m / params.lateral_sample_step_m - EPSILON));
    const double away_sign = (!obstacles.empty() && obstacles.front().d >= 0.0) ? -1.0 : 1.0;

    std::vector<double> targets{0.0};
    for (int k = 1; k <= step_count; ++k) {
        const double magnitude = std::min(k * params.lateral_sample_step_m, params.max_shift_m);
        targets.push_back(away_sign * magnitude);
        targets.push_back(-away_sign * magnitude);
    }
    return targets;
}

std::vector<Pose2D> select_best_path(
    const ReferencePath & reference, const FrenetState & initial, const double end_s,
    const std::vector<double> & target_s_list, const std::vector<double> & target_d_list,
    const std::vector<FrenetObstacle> & obstacles, const std::vector<Pose2D> & previous_points,
    const Parameters & params)
{
    struct Candidate
    {
        double target_d;
        std::vector<Pose2D> points;
        double clearance;
        bool curvature_ok;
        double cost;
    };

    const double interval = params.resample_interval_m;
    const double survival_margin = params.tread_m * 0.5 + params.collision_check_margin_m;
    std::vector<Candidate> candidates;
    for (const double target_s : target_s_list) {
        for (const double target_d : target_d_list) {
            std::vector<double> s_points;
            std::vector<double> offsets;
            sample_quintic(initial, target_s, target_d, interval, s_points, offsets);
            if (s_points.size() < 2U) {
                continue;
            }
            // 合流後は横位置を保ったまま end_s まで延ばす
            const double hold = offsets.back();
            for (double s = s_points.back() + interval; s <= end_s + EPSILON; s += interval) {
                s_points.push_back(s);
                offsets.push_back(hold);
            }

            // 障害物との横方向の余裕
            double clearance = std::numeric_limits<double>::infinity();
            for (const auto & obstacle : obstacles) {
                for (std::size_t i = 0U; i < s_points.size(); ++i) {
                    if (std::abs(s_points[i] - obstacle.s) <= COLLISION_LONGITUDINAL_MARGIN_M + obstacle.half_length) {
                        clearance = std::min(clearance, std::abs(offsets[i] - obstacle.d) - obstacle.half_width);
                    }
                }
            }
            if (clearance <= survival_margin) {
                continue;
            }

            auto points = reference.to_cartesian(s_points, offsets);
            double curvature_sum = 0.0;
            bool curvature_ok = true;
            for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
                const double segment = std::hypot(points[i].x - points[i - 1U].x, points[i].y - points[i - 1U].y);
                const double curvature = std::abs(std::remainder(points[i].yaw - points[i - 1U].yaw, 2.0 * M_PI) / segment);
                curvature_sum += curvature;
                curvature_ok = curvature_ok && curvature <= params.kappa_max;
            }
            const double curvature_cost = points.size() > 2U ?
                params.weight_curvature * curvature_sum / static_cast<double>(points.size() - 2U) : 0.0;
            const double change = obstacles.empty() ? 0.0 : mean_deviation(points, previous_points);
            const double cost = curvature_cost + params.weight_lateral_deviation * std::abs(hold) +
                params.weight_side_change * change;
            candidates.push_back(Candidate{target_d, std::move(points), clearance, curvature_ok, cost});
        }
    }

    // 余裕 hard_margin_m を確保できる候補を優先し、なければ衝突しないだけの候補から選ぶ
    for (const double margin : {survival_margin + params.hard_margin_m, survival_margin}) {
        const Candidate * best = nullptr;
        for (const auto & candidate : candidates) {
            if (candidate.curvature_ok && candidate.clearance > margin && (!best || candidate.cost < best->cost)) {
                best = &candidate;
            }
        }
        if (best) {
            return best->points;
        }
    }
    // 曲率制約を満たす候補がなければ、回避シフトなしの経路を使う
    for (const auto & candidate : candidates) {
        if (candidate.target_d == 0.0) {
            return candidate.points;
        }
    }
    return {};
}

}
