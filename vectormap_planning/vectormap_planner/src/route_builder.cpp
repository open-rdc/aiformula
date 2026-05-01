#include "vectormap_planner/vectormap_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace vectormap_planner
{
namespace
{
using Lanelet = vectormap_msgs::msg::Lanelet;
using LineString = vectormap_msgs::msg::LineString;

constexpr double EPSILON = 1.0e-6;

double distance_2d(const VectormapPlannerNode::Point2D& a, const VectormapPlannerNode::Point2D& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

VectormapPlannerNode::PathPoint interpolate_raw_path(
    const std::vector<VectormapPlannerNode::Point2D>& points,
    const std::vector<double>& s_values,
    const double target_s,
    const uint64_t lanelet_id)
{
    if (points.size() != s_values.size() || points.size() < 2U) {
        throw std::runtime_error("raw path is invalid");
    }

    const double clamped_s = std::clamp(target_s, s_values.front(), s_values.back());
    const auto upper = std::upper_bound(s_values.begin(), s_values.end(), clamped_s);
    std::size_t index = 0U;
    if (upper == s_values.end()) {
        index = s_values.size() - 2U;
    } else if (upper != s_values.begin()) {
        index = static_cast<std::size_t>(std::distance(s_values.begin(), upper) - 1);
    }

    const double segment_length = s_values[index + 1U] - s_values[index];
    const double ratio = segment_length > EPSILON ? (clamped_s - s_values[index]) / segment_length : 0.0;
    const auto& start = points[index];
    const auto& end = points[index + 1U];
    const double x = start.x + ratio * (end.x - start.x);
    const double y = start.y + ratio * (end.y - start.y);
    const double yaw = std::atan2(end.y - start.y, end.x - start.x);
    return VectormapPlannerNode::PathPoint{clamped_s, x, y, yaw, lanelet_id};
}

}  // namespace

void VectormapPlannerNode::build_global_path_once(const vectormap_msgs::msg::VectorMap& map_msg)
{
    if (map_msg.header.frame_id != map_frame_id_) {
        throw std::runtime_error("VectorMap frame_id must be " + map_frame_id_ + ", got " + map_msg.header.frame_id);
    }

    std::unordered_map<uint64_t, Lanelet> lanelet_by_id;
    lanelet_by_id.reserve(map_msg.lanelets.size());
    for (const auto& lanelet : map_msg.lanelets) {
        lanelet_by_id.emplace(lanelet.id, lanelet);
    }

    std::unordered_map<uint64_t, LineString> line_string_by_id;
    line_string_by_id.reserve(map_msg.line_strings.size());
    for (const auto& line_string : map_msg.line_strings) {
        line_string_by_id.emplace(line_string.id, line_string);
    }

    std::unordered_map<uint64_t, std::vector<uint64_t>> connections_by_from;
    connections_by_from.reserve(map_msg.lane_connections.size());
    for (const auto& connection : map_msg.lane_connections) {
        connections_by_from[connection.from_lanelet_id].push_back(connection.to_lanelet_id);
    }

    build_adjacency(lanelet_by_id);

    lanelet_centerline_points_by_id_.clear();
    lanelet_centerline_points_by_id_.reserve(lanelet_by_id.size());
    for (const auto& [lanelet_id, lanelet] : lanelet_by_id) {
        const auto line_it = line_string_by_id.find(lanelet.centerline_id);
        if (line_it == line_string_by_id.end() || line_it->second.points.size() < 2U) {
            continue;
        }
        std::vector<Point2D> centerline_points;
        centerline_points.reserve(line_it->second.points.size());
        for (const auto& point : line_it->second.points) {
            centerline_points.push_back(Point2D{point.x, point.y});
        }
        lanelet_centerline_points_by_id_.emplace(lanelet_id, std::move(centerline_points));
    }

    std::vector<Point2D> route_points;
    std::vector<double> raw_s;
    lanelet_ranges_.clear();
    route_is_loop_ = false;
    route_points.reserve(route_lanelet_ids_param_.size() * 16U);

    double accumulated_s = 0.0;
    for (std::size_t route_index = 0U; route_index < route_lanelet_ids_param_.size(); ++route_index) {
        const uint64_t lanelet_id = static_cast<uint64_t>(route_lanelet_ids_param_[route_index]);
        const auto lanelet_it = lanelet_by_id.find(lanelet_id);
        if (lanelet_it == lanelet_by_id.end()) {
            throw std::runtime_error("route lanelet id not found: " + std::to_string(lanelet_id));
        }

        if (route_index + 1U < route_lanelet_ids_param_.size()) {
            const uint64_t next_id = static_cast<uint64_t>(route_lanelet_ids_param_[route_index + 1U]);
            const auto connections_it = connections_by_from.find(lanelet_id);
            const bool connected = connections_it != connections_by_from.end() &&
                std::find(connections_it->second.begin(), connections_it->second.end(), next_id) !=
                    connections_it->second.end();
            if (!connected) {
                throw std::runtime_error(
                    "missing LaneConnection from " + std::to_string(lanelet_id) + " to " + std::to_string(next_id));
            }
        }

        const auto line_it = line_string_by_id.find(lanelet_it->second.centerline_id);
        if (line_it == line_string_by_id.end()) {
            throw std::runtime_error("centerline id not found: " + std::to_string(lanelet_it->second.centerline_id));
        }
        if (line_it->second.points.size() < 2U) {
            throw std::runtime_error("centerline must contain at least two points");
        }
        if (line_it->second.line_type != LineString::TYPE_VIRTUAL_LINE ||
            line_it->second.marking_type != LineString::MARKING_VIRTUAL)
        {
            throw std::runtime_error("route centerline must be virtual line");
        }

        const auto centerline_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (centerline_it == lanelet_centerline_points_by_id_.end()) {
            throw std::runtime_error("route centerline points are not available");
        }
        const auto& centerline_points = centerline_it->second;
        if (!route_points.empty()) {
            const double gap = distance_2d(route_points.back(), centerline_points.front());
            if (gap > max_centerline_connection_gap_m_) {
                throw std::runtime_error("centerline connection gap is too large: " + std::to_string(gap));
            }
        }

        const double lanelet_start_s = accumulated_s;
        for (const auto& point : centerline_points) {
            if (!route_points.empty()) {
                const double ds = distance_2d(route_points.back(), point);
                if (ds <= EPSILON) {
                    continue;
                }
                accumulated_s += ds;
            }
            route_points.push_back(point);
            raw_s.push_back(accumulated_s);
        }
        lanelet_ranges_.push_back(LaneletRange{lanelet_id, lanelet_start_s, accumulated_s});
    }

    if (route_points.size() < 2U) {
        throw std::runtime_error("route must contain at least two centerline points");
    }

    const uint64_t first_lanelet_id = static_cast<uint64_t>(route_lanelet_ids_param_.front());
    const uint64_t last_lanelet_id = static_cast<uint64_t>(route_lanelet_ids_param_.back());
    const auto last_connections_it = connections_by_from.find(last_lanelet_id);
    route_is_loop_ = last_connections_it != connections_by_from.end() &&
        std::find(last_connections_it->second.begin(), last_connections_it->second.end(), first_lanelet_id) !=
            last_connections_it->second.end();
    if (route_is_loop_) {
        const double closing_gap = distance_2d(route_points.back(), route_points.front());
        if (closing_gap > max_centerline_connection_gap_m_) {
            throw std::runtime_error("loop centerline closing gap is too large: " + std::to_string(closing_gap));
        }
        if (closing_gap > EPSILON) {
            accumulated_s += closing_gap;
            route_points.push_back(route_points.front());
            raw_s.push_back(accumulated_s);
            lanelet_ranges_.back().end_s = accumulated_s;
        }
    }

    global_samples_.clear();
    global_samples_.reserve(static_cast<std::size_t>(std::ceil(raw_s.back() / global_path_resample_interval_m_)) + 2U);
    for (double s = 0.0; s < raw_s.back(); s += global_path_resample_interval_m_) {
        global_samples_.push_back(interpolate_raw_path(route_points, raw_s, s, lanelet_at_s(s)));
    }
    global_samples_.push_back(interpolate_raw_path(route_points, raw_s, raw_s.back(), lanelet_at_s(raw_s.back())));

    path_frame_id_ = map_msg.header.frame_id;
    global_path_ready_ = true;
    RCLCPP_INFO(
        get_logger(),
        "built vector map global path: %zu points, loop=%s",
        global_samples_.size(),
        route_is_loop_ ? "true" : "false");
}

void VectormapPlannerNode::build_adjacency(
    const std::unordered_map<uint64_t, Lanelet>& lanelet_by_id)
{
    left_adjacent_lanelet_by_id_.clear();
    right_adjacent_lanelet_by_id_.clear();
    for (const auto& [id, lanelet] : lanelet_by_id) {
        for (const auto& [other_id, other] : lanelet_by_id) {
            if (id == other_id) {
                continue;
            }
            if (lanelet.left_line_id == other.right_line_id) {
                left_adjacent_lanelet_by_id_[id] = other_id;
            }
            if (lanelet.right_line_id == other.left_line_id) {
                right_adjacent_lanelet_by_id_[id] = other_id;
            }
        }
    }
}

}  // namespace vectormap_planner
