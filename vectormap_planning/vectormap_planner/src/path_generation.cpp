#include "vectormap_planner/vectormap_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vectormap_planner
{
namespace
{

constexpr double EPSILON = 1.0e-6;

}  // namespace

void VectormapPlannerNode::start_lane_change(const double current_s, const uint64_t current_lanelet_id)
{
    if (lane_change_state_ == LaneChangeState::Executing) {
        RCLCPP_WARN(get_logger(), "lane change request ignored because another lane change is executing");
        return;
    }

    const bool to_left = lane_change_direction_ == "left";
    const auto& adjacency = to_left ? left_adjacent_lanelet_by_id_ : right_adjacent_lanelet_by_id_;
    if (adjacency.find(current_lanelet_id) == adjacency.end()) {
        RCLCPP_WARN(get_logger(), "lane change rejected: no adjacent lanelet from %lu", current_lanelet_id);
        return;
    }

    const double target_offset = adjacent_lane_offset(current_lanelet_id, to_left);
    if (std::abs(target_offset) <= EPSILON) {
        RCLCPP_WARN(get_logger(), "lane change rejected: adjacent lane offset is zero");
        return;
    }

    lane_change_state_ = LaneChangeState::Executing;
    lane_change_start_s_ = current_s;
    lane_change_end_s_ = route_is_loop_ ? current_s + lane_change_length_m_ : std::min(current_s + lane_change_length_m_, max_path_s());
    lane_change_start_offset_m_ = active_lane_offset_m_;
    lane_change_target_offset_m_ = target_offset;
    RCLCPP_INFO(
        get_logger(),
        "lane change started: lanelet=%lu direction=%s target_offset=%.3f",
        current_lanelet_id,
        lane_change_direction_.c_str(),
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
    const double current_s)
{
    double start_offset = active_lane_offset_m_;
    double end_offset = active_lane_offset_m_;
    double start_s = lane_change_start_s_;
    double end_s = lane_change_end_s_;
    if (lane_change_state_ == LaneChangeState::Executing && current_s >= lane_change_end_s_) {
        active_lane_offset_m_ = lane_change_target_offset_m_;
        lane_change_state_ = LaneChangeState::Idle;
        start_offset = active_lane_offset_m_;
        end_offset = active_lane_offset_m_;
        start_s = current_s;
        end_s = current_s;
        RCLCPP_INFO(get_logger(), "lane change completed: active_offset=%.3f", active_lane_offset_m_);
    } else if (lane_change_state_ == LaneChangeState::Executing) {
        start_offset = lane_change_start_offset_m_;
        end_offset = lane_change_target_offset_m_;
    }

    return sample_shifted_path(
        current_s,
        route_is_loop_ ? current_s + local_path_length_m_ : std::min(current_s + local_path_length_m_, max_path_s()),
        active_lane_offset_m_,
        start_s,
        end_s,
        start_offset,
        end_offset,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
}

std::vector<VectormapPlannerNode::PathPoint> VectormapPlannerNode::sample_shifted_path(
    const double start_s,
    const double end_s,
    const double base_offset,
    const double lane_change_start_s,
    const double lane_change_end_s,
    const double lane_change_start_offset,
    const double lane_change_end_offset,
    const double avoidance_obstacle_s,
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

    for (double s = start_s; s < end_s; s += local_path_resample_interval_m_) {
        const auto base_point = path_point_at_s(s);
        double offset = base_offset;
        if (lane_change_end_s > lane_change_start_s && s >= lane_change_start_s) {
            const double t = std::clamp((s - lane_change_start_s) / (lane_change_end_s - lane_change_start_s), 0.0, 1.0);
            offset = lane_change_start_offset +
                smooth_step(t) * (lane_change_end_offset - lane_change_start_offset);
        }
        if (std::isfinite(avoidance_obstacle_s)) {
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

    const auto base_point = path_point_at_s(end_s);
    points.push_back(PathPoint{end_s, base_point.x, base_point.y, base_point.yaw, base_point.lanelet_id});
    return points;
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
        return 0.0;
    }

    const auto target_it = lanelet_centerline_points_by_id_.find(adjacent_it->second);
    if (target_it == lanelet_centerline_points_by_id_.end() || target_it->second.empty()) {
        return 0.0;
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
        return to_left ? 2.0 : -2.0;
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
