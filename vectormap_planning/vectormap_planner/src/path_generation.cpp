#include "vectormap_planner/vectormap_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vectormap_planner
{
namespace
{

constexpr double EPSILON = 1.0e-6;

}  // namespace

void VectormapPlannerNode::start_lane_switch(const double current_s, const uint64_t current_lanelet_id)
{
    if (std::abs(active_lane_offset_m_) > EPSILON) {
        start_lane_offset_transition(current_s, 0.0, "return_to_route_lane");
        return;
    }

    const bool has_left = left_adjacent_lanelet_by_id_.find(current_lanelet_id) != left_adjacent_lanelet_by_id_.end();
    const bool has_right = right_adjacent_lanelet_by_id_.find(current_lanelet_id) != right_adjacent_lanelet_by_id_.end();
    if (has_left && has_right) {
        RCLCPP_ERROR(
            get_logger(),
            "lane switch rejected: both left and right adjacent lanelets exist from %lu; expected two-lane route",
            current_lanelet_id);
        return;
    }
    if (!has_left && !has_right) {
        RCLCPP_WARN(get_logger(), "lane switch rejected: no adjacent lanelet from %lu", current_lanelet_id);
        return;
    }

    const bool to_left = has_left;
    const double target_offset = adjacent_lane_offset(current_lanelet_id, to_left);
    if (!std::isfinite(target_offset) || std::abs(target_offset) <= EPSILON) {
        RCLCPP_WARN(get_logger(), "lane switch rejected: adjacent lane offset is invalid");
        return;
    }

    start_lane_offset_transition(current_s, target_offset, "switch_to_adjacent_lane");
}

void VectormapPlannerNode::start_lane_offset_transition(
    const double current_s,
    const double target_offset,
    const std::string& reason)
{
    if (lane_change_state_ == LaneChangeState::Executing) {
        RCLCPP_WARN(get_logger(), "lane switch request ignored because another lane change is executing");
        return;
    }
    if (!std::isfinite(target_offset)) {
        throw std::runtime_error("lane switch target offset must be finite");
    }
    if (std::abs(target_offset - active_lane_offset_m_) <= EPSILON) {
        RCLCPP_WARN(get_logger(), "lane switch request ignored because target offset is already active");
        return;
    }

    lane_change_state_ = LaneChangeState::Executing;
    lane_change_start_s_ = current_s;
    lane_change_end_s_ = route_is_loop_ ? current_s + lane_change_length_m_ : std::min(current_s + lane_change_length_m_, max_path_s());
    lane_change_start_offset_m_ = active_lane_offset_m_;
    lane_change_target_offset_m_ = target_offset;
    RCLCPP_INFO(
        get_logger(),
        "lane switch started: reason=%s start_offset=%.3f target_offset=%.3f",
        reason.c_str(),
        lane_change_start_offset_m_,
        lane_change_target_offset_m_);
}

nav_msgs::msg::Path VectormapPlannerNode::make_path_message(
    const std::vector<PathPoint>& points,
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = path_frame_id_.empty() ? map_frame_id_ : path_frame_id_;
    path.poses.reserve(points.size());
    for (const auto& point : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = yaw_to_quaternion(point.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

std::vector<VectormapPlannerNode::PathPoint> VectormapPlannerNode::generate_local_path(
    const FrenetPoint& ego_frenet,
    const bool has_obstacle,
    const FrenetObstacle& obstacle,
    const double avoidance_shift,
    const double speed_mps)
{
    const double current_s = ego_frenet.s;
    double target_offset = active_lane_offset_m_;
    double lane_start_offset = active_lane_offset_m_;
    double lane_end_offset = active_lane_offset_m_;
    double start_s = lane_change_start_s_;
    double end_s = lane_change_end_s_;
    if (lane_change_state_ == LaneChangeState::Executing && current_s >= lane_change_end_s_) {
        active_lane_offset_m_ = lane_change_target_offset_m_;
        lane_change_state_ = LaneChangeState::Idle;
        lane_switch_completed_ = true;
        target_offset = active_lane_offset_m_;
        lane_start_offset = active_lane_offset_m_;
        lane_end_offset = active_lane_offset_m_;
        start_s = current_s;
        end_s = current_s;
        RCLCPP_INFO(get_logger(), "lane change completed: active_offset=%.3f", active_lane_offset_m_);
    } else if (lane_change_state_ == LaneChangeState::Executing) {
        target_offset = lane_change_target_offset_m_;
        lane_start_offset = lane_change_start_offset_m_;
        lane_end_offset = lane_change_target_offset_m_;
    }

    const double path_end_s =
        route_is_loop_ ? current_s + local_path_horizon_m_ : std::min(current_s + local_path_horizon_m_, max_path_s());
    if (path_end_s <= current_s + EPSILON) {
        return {};
    }

    double avoidance_start_s = 0.0;
    double avoidance_end_s = 0.0;
    double avoidance_return_start_s = 0.0;
    double avoidance_return_end_s = 0.0;
    if (has_obstacle) {
        const double longitudinal_distance =
            4.0 * speed_mps * std::cbrt(0.5 * std::abs(avoidance_shift) / avoidance_lateral_jerk_mps3_);
        avoidance_start_s = std::max(current_s, obstacle.s - longitudinal_distance);
        avoidance_end_s = std::max(avoidance_start_s + local_path_resample_interval_m_, obstacle.s - 0.5);
        avoidance_return_start_s = obstacle.s + 2.0;
        avoidance_return_end_s = avoidance_return_start_s + longitudinal_distance;
    }

    std::vector<double> candidate_shifts;
    candidate_shifts.reserve(3U);
    candidate_shifts.push_back(has_obstacle ? avoidance_shift : 0.0);
    if (has_obstacle) {
        candidate_shifts.push_back(-avoidance_shift);
        candidate_shifts.push_back(0.0);
    }

    double best_cost = std::numeric_limits<double>::max();
    std::vector<PathPoint> best_path;
    for (const double candidate_shift : candidate_shifts) {
        auto candidate = sample_frenet_path(
            current_s,
            path_end_s,
            ego_frenet.d,
            start_s,
            end_s,
            lane_start_offset,
            lane_end_offset,
            target_offset,
            obstacle,
            candidate_shift,
            avoidance_start_s,
            avoidance_end_s,
            avoidance_return_start_s,
            avoidance_return_end_s);
        if (candidate.empty()) {
            continue;
        }
        if (has_obstacle && !is_collision_free(candidate, obstacle)) {
            continue;
        }
        const double cost = evaluate_frenet_candidate(candidate, target_offset, candidate_shift);
        if (cost < best_cost) {
            best_cost = cost;
            best_path = std::move(candidate);
        }
    }

    return best_path;
}

std::vector<VectormapPlannerNode::PathPoint> VectormapPlannerNode::sample_frenet_path(
    const double start_s,
    const double end_s,
    const double start_d,
    const double lane_change_start_s,
    const double lane_change_end_s,
    const double lane_change_start_offset,
    const double lane_change_end_offset,
    const double fallback_target_offset,
    const FrenetObstacle& obstacle,
    const double avoidance_shift,
    const double avoidance_start_s,
    const double avoidance_end_s,
    const double avoidance_return_start_s,
    const double avoidance_return_end_s) const
{
    std::vector<PathPoint> points;
    if (end_s <= start_s) {
        return points;
    }
    points.reserve(static_cast<std::size_t>(std::ceil((end_s - start_s) / local_path_resample_interval_m_)) + 2U);

    const double convergence_length = std::min(5.0, std::max(local_path_resample_interval_m_, end_s - start_s));
    for (double s = start_s; s < end_s; s += local_path_resample_interval_m_) {
        const auto base_point = path_point_at_s(s);
        double target_offset = fallback_target_offset;
        if (lane_change_end_s > lane_change_start_s && s >= lane_change_start_s) {
            const double t = std::clamp((s - lane_change_start_s) / (lane_change_end_s - lane_change_start_s), 0.0, 1.0);
            target_offset = lane_change_start_offset +
                smooth_step(t) * (lane_change_end_offset - lane_change_start_offset);
        }

        const double converge_t = std::clamp((s - start_s) / convergence_length, 0.0, 1.0);
        double offset = start_d + smooth_step(converge_t) * (target_offset - start_d);
        if (std::isfinite(obstacle.s)) {
            if (s >= avoidance_start_s && s < avoidance_end_s) {
                const double t = std::clamp((s - avoidance_start_s) / (avoidance_end_s - avoidance_start_s), 0.0, 1.0);
                offset += smooth_step(t) * avoidance_shift;
            } else if (s >= avoidance_end_s && s < avoidance_return_start_s) {
                offset += avoidance_shift;
            } else if (s >= avoidance_return_start_s && s < avoidance_return_end_s) {
                const double t = std::clamp(
                    (s - avoidance_return_start_s) / (avoidance_return_end_s - avoidance_return_start_s),
                    0.0,
                    1.0);
                offset += (1.0 - smooth_step(t)) * avoidance_shift;
            }
        }

        points.push_back(PathPoint{
            s,
            base_point.x - std::sin(base_point.yaw) * offset,
            base_point.y + std::cos(base_point.yaw) * offset,
            base_point.yaw,
            base_point.lanelet_id});
    }

    if (!points.empty() && end_s - points.back().s > EPSILON) {
        const auto base_point = path_point_at_s(end_s);
        const double offset = points.back().s < end_s ? project_to_path(Point2D{points.back().x, points.back().y}).d : start_d;
        points.push_back(PathPoint{
            end_s,
            base_point.x - std::sin(base_point.yaw) * offset,
            base_point.y + std::cos(base_point.yaw) * offset,
            base_point.yaw,
            base_point.lanelet_id});
    }

    for (std::size_t i = 1U; i < points.size(); ++i) {
        const double dx = points[i].x - points[i - 1U].x;
        const double dy = points[i].y - points[i - 1U].y;
        if (std::hypot(dx, dy) > EPSILON) {
            points[i - 1U].yaw = std::atan2(dy, dx);
        }
    }
    if (points.size() >= 2U) {
        points.back().yaw = points[points.size() - 2U].yaw;
    }
    return points;
}

bool VectormapPlannerNode::is_collision_free(
    const std::vector<PathPoint>& candidate,
    const FrenetObstacle& obstacle) const
{
    if (!std::isfinite(obstacle.s) || !std::isfinite(obstacle.d)) {
        return true;
    }
    const double longitudinal_margin = std::max(0.5, local_path_resample_interval_m_ * 2.0);
    const double lateral_margin =
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ + envelope_buffer_margin_m_ + frenet_collision_check_margin_m_;
    for (const auto& point : candidate) {
        if (std::abs(point.s - obstacle.s) > longitudinal_margin) {
            continue;
        }
        const FrenetPoint frenet = project_to_path(Point2D{point.x, point.y});
        if (std::abs(frenet.d - obstacle.d) <= lateral_margin) {
            return false;
        }
    }
    return true;
}

double VectormapPlannerNode::evaluate_frenet_candidate(
    const std::vector<PathPoint>& candidate,
    const double target_offset,
    const double avoidance_shift) const
{
    if (candidate.empty()) {
        return std::numeric_limits<double>::max();
    }

    double lateral_change_sum = 0.0;
    double previous_d = project_to_path(Point2D{candidate.front().x, candidate.front().y}).d;
    for (std::size_t i = 1U; i < candidate.size(); ++i) {
        const double d = project_to_path(Point2D{candidate[i].x, candidate[i].y}).d;
        lateral_change_sum += std::abs(d - previous_d);
        previous_d = d;
    }
    const double final_d = previous_d;
    return frenet_weight_lateral_offset_ * std::abs(final_d - target_offset) +
        frenet_weight_lateral_change_ * lateral_change_sum +
        frenet_weight_avoidance_shift_ * std::abs(avoidance_shift);
}

VectormapPlannerNode::FrenetPoint VectormapPlannerNode::project_to_path(const Point2D& point) const
{
    if (global_samples_.empty()) {
        throw std::runtime_error("global path is not ready");
    }

    double best_distance_sq = std::numeric_limits<double>::max();
    FrenetPoint best{global_samples_.front().s, 0.0};
    for (std::size_t i = 1U; i < global_samples_.size(); ++i) {
        const auto& a = global_samples_[i - 1U];
        const auto& b = global_samples_[i];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double length_sq = vx * vx + vy * vy;
        if (length_sq <= EPSILON) {
            continue;
        }
        const double wx = point.x - a.x;
        const double wy = point.y - a.y;
        const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
        const double px = a.x + t * vx;
        const double py = a.y + t * vy;
        const double dx = point.x - px;
        const double dy = point.y - py;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            const double yaw = std::atan2(vy, vx);
            best.s = a.s + t * (b.s - a.s);
            best.d = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        }
    }
    return best;
}

VectormapPlannerNode::PathPoint VectormapPlannerNode::path_point_at_s(const double s) const
{
    if (global_samples_.size() < 2U) {
        throw std::runtime_error("global path is not ready");
    }

    const double clamped_s = normalize_path_s(s);
    const auto upper = std::upper_bound(
        global_samples_.begin(),
        global_samples_.end(),
        clamped_s,
        [](const double value, const PathPoint& point) {
            return value < point.s;
        });
    std::size_t index = 0U;
    if (upper == global_samples_.end()) {
        index = global_samples_.size() - 2U;
    } else if (upper != global_samples_.begin()) {
        index = static_cast<std::size_t>(std::distance(global_samples_.begin(), upper) - 1);
    }

    const auto& start = global_samples_[index];
    const auto& end = global_samples_[index + 1U];
    const double segment_length = end.s - start.s;
    const double ratio = segment_length > EPSILON ? (clamped_s - start.s) / segment_length : 0.0;
    return PathPoint{
        clamped_s,
        start.x + ratio * (end.x - start.x),
        start.y + ratio * (end.y - start.y),
        std::atan2(end.y - start.y, end.x - start.x),
        lanelet_at_s(clamped_s)};
}

double VectormapPlannerNode::max_path_s() const
{
    if (global_samples_.empty()) {
        throw std::runtime_error("global path is not ready");
    }
    return global_samples_.back().s;
}

double VectormapPlannerNode::normalize_path_s(const double s) const
{
    double path_length = 0.0;
    if (!global_samples_.empty()) {
        path_length = global_samples_.back().s;
    } else if (!lanelet_ranges_.empty()) {
        path_length = lanelet_ranges_.back().end_s;
    }

    if (path_length <= EPSILON) {
        return s;
    }
    if (!route_is_loop_) {
        return std::clamp(s, 0.0, path_length);
    }

    double normalized = std::fmod(s, path_length);
    if (normalized < 0.0) {
        normalized += path_length;
    }
    return normalized;
}

uint64_t VectormapPlannerNode::lanelet_at_s(const double s) const
{
    if (lanelet_ranges_.empty()) {
        return 0U;
    }
    const double normalized_s = normalize_path_s(s);
    const auto it = std::find_if(
        lanelet_ranges_.begin(),
        lanelet_ranges_.end(),
        [normalized_s](const LaneletRange& range) {
            return normalized_s >= range.start_s && normalized_s <= range.end_s;
        });
    if (it != lanelet_ranges_.end()) {
        return it->lanelet_id;
    }
    return lanelet_ranges_.back().lanelet_id;
}

double VectormapPlannerNode::adjacent_lane_offset(const uint64_t lanelet_id, const bool to_left) const
{
    const auto& adjacency = to_left ? left_adjacent_lanelet_by_id_ : right_adjacent_lanelet_by_id_;
    const auto adjacent_it = adjacency.find(lanelet_id);
    if (adjacent_it == adjacency.end()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const auto target_it = lanelet_centerline_points_by_id_.find(adjacent_it->second);
    if (target_it == lanelet_centerline_points_by_id_.end() || target_it->second.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double offset_sum = 0.0;
    std::size_t count = 0U;
    for (const auto& target_point : target_it->second) {
        const FrenetPoint projected = project_to_path(target_point);
        if (lanelet_at_s(projected.s) == lanelet_id) {
            offset_sum += projected.d;
            ++count;
        }
    }
    if (count == 0U) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return offset_sum / static_cast<double>(count);
}

double VectormapPlannerNode::smooth_step(const double t)
{
    const double x = std::clamp(t, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

geometry_msgs::msg::Quaternion VectormapPlannerNode::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace vectormap_planner
