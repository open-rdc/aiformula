#include "lane_planner/lane_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace lane_planner
{
namespace
{
using Lanelet = vectormap_msgs::msg::Lanelet;
using LineString = vectormap_msgs::msg::LineString;

constexpr double EPSILON = 1.0e-6;

double distance_2d(
    const LanePlannerNode::Point2D& a,
    const LanePlannerNode::Point2D& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double point_segment_distance_sq(
    const LanePlannerNode::Point2D& point,
    const LanePlannerNode::Point2D& start,
    const LanePlannerNode::Point2D& end)
{
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= EPSILON) {
        const double dx = point.x - start.x;
        const double dy = point.y - start.y;
        return dx * dx + dy * dy;
    }
    const double wx = point.x - start.x;
    const double wy = point.y - start.y;
    const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
    const double px = start.x + t * vx;
    const double py = start.y + t * vy;
    const double dx = point.x - px;
    const double dy = point.y - py;
    return dx * dx + dy * dy;
}

LanePlannerNode::PathPoint interpolate_raw_path(
    const std::vector<LanePlannerNode::Point2D>& points,
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
    return LanePlannerNode::PathPoint{clamped_s, x, y, yaw, lanelet_id};
}

}  // namespace

void LanePlannerNode::build_global_path_once(
    const vectormap_msgs::msg::VectorMap& map_msg)
{
    if (map_msg.header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "VectorMap frame_id must be " + map_frame_id_ + ", got " + map_msg.header.frame_id);
    }

    lanelet_by_id_.clear();
    lanelet_by_id_.reserve(map_msg.lanelets.size());
    for (const auto& lanelet : map_msg.lanelets) {
        lanelet_by_id_.emplace(lanelet.id, lanelet);
    }

    std::unordered_map<uint64_t, LineString> line_string_by_id;
    line_string_by_id.reserve(map_msg.line_strings.size());
    for (const auto& line_string : map_msg.line_strings) {
        line_string_by_id.emplace(line_string.id, line_string);
    }

    connection_edges_by_from_lanelet_id_.clear();
    connection_edges_by_from_lanelet_id_.reserve(map_msg.lane_connections.size());
    for (const auto& connection : map_msg.lane_connections) {
        connection_edges_by_from_lanelet_id_[connection.from_lanelet_id].push_back(
            RouteEdge{connection.to_lanelet_id, connection.turn_direction, connection.cost});
    }

    lanelet_centerline_points_by_id_.clear();
    lanelet_centerline_points_by_id_.reserve(lanelet_by_id_.size());
    for (const auto& [lanelet_id, lanelet] : lanelet_by_id_) {
        const auto line_it = line_string_by_id.find(lanelet.centerline_id);
        if (line_it == line_string_by_id.end() || line_it->second.points.size() < 2U) {
            continue;
        }
        if (line_it->second.line_type != LineString::TYPE_VIRTUAL_LINE ||
            line_it->second.marking_type != LineString::MARKING_VIRTUAL)
        {
            throw std::runtime_error("route centerline must be virtual line");
        }
        std::vector<Point2D> centerline_points;
        centerline_points.reserve(line_it->second.points.size());
        for (const auto& point : line_it->second.points) {
            centerline_points.push_back(Point2D{point.x, point.y});
        }
        lanelet_centerline_points_by_id_.emplace(lanelet_id, std::move(centerline_points));
    }

    std::size_t fallback_count = 0U;
    const auto route_lanelet_ids = build_route_sequence_from_graph(
        static_cast<uint64_t>(route_lanelet_ids_param_.front()),
        fallback_count);
    if (fallback_count > 0U) {
        RCLCPP_WARN(
            get_logger(),
            "initial route used nav_cmd fallback %zu times: requested=%s",
            fallback_count,
            turn_direction_to_string(last_nav_cmd_turn_).c_str());
    }
    build_route_from_lanelet_ids(route_lanelet_ids);
    path_frame_id_ = map_msg.header.frame_id;
    global_path_ready_ = true;
    RCLCPP_INFO(
        get_logger(),
        "built vector map global path: %zu points, route_lanelets=%zu, loop=%s",
        global_samples_.size(),
        route_lanelet_ids.size(),
        route_is_loop_ ? "true" : "false");
}

void LanePlannerNode::build_route_from_lanelet_ids(
    const std::vector<uint64_t>& route_lanelet_ids)
{
    if (route_lanelet_ids.empty()) {
        throw std::runtime_error("route lanelet sequence must not be empty");
    }

    std::vector<Point2D> route_points;
    std::vector<double> raw_s;
    lanelet_ranges_.clear();
    route_is_loop_ = false;
    route_points.reserve(route_lanelet_ids.size() * 16U);

    double accumulated_s = 0.0;
    for (std::size_t route_index = 0U; route_index < route_lanelet_ids.size(); ++route_index) {
        const uint64_t lanelet_id = route_lanelet_ids[route_index];
        const auto lanelet_it = lanelet_by_id_.find(lanelet_id);
        if (lanelet_it == lanelet_by_id_.end()) {
            throw std::runtime_error(
                "route lanelet id not found: " + std::to_string(lanelet_id));
        }

        if (route_index + 1U < route_lanelet_ids.size()) {
            const uint64_t next_id = route_lanelet_ids[route_index + 1U];
            const auto connections_it = connection_edges_by_from_lanelet_id_.find(lanelet_id);
            const bool connected = connections_it != connection_edges_by_from_lanelet_id_.end() &&
                std::find_if(
                    connections_it->second.begin(),
                    connections_it->second.end(),
                    [next_id](const RouteEdge& edge) {
                        return edge.to_lanelet_id == next_id;
                    }) != connections_it->second.end();
            if (!connected) {
                throw std::runtime_error(
                    "missing LaneConnection from " + std::to_string(lanelet_id) +
                    " to " + std::to_string(next_id));
            }
        }

        const auto centerline_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (centerline_it == lanelet_centerline_points_by_id_.end()) {
            throw std::runtime_error("route centerline points are not available");
        }
        const auto& centerline_points = centerline_it->second;
        if (!route_points.empty()) {
            const double gap = distance_2d(route_points.back(), centerline_points.front());
            if (gap > max_centerline_connection_gap_m_) {
                throw std::runtime_error(
                    "centerline connection gap is too large: " + std::to_string(gap));
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

    const uint64_t first_lanelet_id = route_lanelet_ids.front();
    const uint64_t last_lanelet_id = route_lanelet_ids.back();
    const auto last_connections_it = connection_edges_by_from_lanelet_id_.find(last_lanelet_id);
    route_is_loop_ = last_connections_it != connection_edges_by_from_lanelet_id_.end() &&
        std::find_if(
            last_connections_it->second.begin(),
            last_connections_it->second.end(),
            [first_lanelet_id](const RouteEdge& edge) {
                return edge.to_lanelet_id == first_lanelet_id;
            }) != last_connections_it->second.end();
    if (route_is_loop_) {
        const double closing_gap = distance_2d(route_points.back(), route_points.front());
        if (closing_gap > max_centerline_connection_gap_m_) {
            throw std::runtime_error(
                "loop centerline closing gap is too large: " + std::to_string(closing_gap));
        }
        if (closing_gap > EPSILON) {
            accumulated_s += closing_gap;
            route_points.push_back(route_points.front());
            raw_s.push_back(accumulated_s);
            lanelet_ranges_.back().end_s = accumulated_s;
        }
    }

    global_samples_.clear();
    global_samples_.reserve(
        static_cast<std::size_t>(std::ceil(raw_s.back() / global_path_resample_interval_m_)) + 2U);
    for (double s = 0.0; s < raw_s.back(); s += global_path_resample_interval_m_) {
        global_samples_.push_back(interpolate_raw_path(route_points, raw_s, s, lanelet_at_s(s)));
    }
    global_samples_.push_back(
        interpolate_raw_path(route_points, raw_s, raw_s.back(), lanelet_at_s(raw_s.back())));
    current_route_lanelet_ids_ = route_lanelet_ids;
}

void LanePlannerNode::rebuild_route_from_lanelet(
    const uint64_t start_lanelet_id,
    const std::string& reason)
{
    std::size_t fallback_count = 0U;
    const auto route_lanelet_ids = build_route_sequence_from_graph(start_lanelet_id, fallback_count);
    build_route_from_lanelet_ids(route_lanelet_ids);
    RCLCPP_INFO(
        get_logger(),
        "rebuilt vector map global path: reason=%s start_lanelet=%lu route_lanelets=%zu points=%zu",
        reason.c_str(),
        start_lanelet_id,
        route_lanelet_ids.size(),
        global_samples_.size());
    if (fallback_count > 0U) {
        RCLCPP_WARN(
            get_logger(),
            "route rebuild used nav_cmd fallback %zu times: requested=%s",
            fallback_count,
            turn_direction_to_string(last_nav_cmd_turn_).c_str());
    }
}

void LanePlannerNode::request_route_rebuild(const std::string& reason)
{
    pending_route_rebuild_ = true;
    if (pending_route_rebuild_reason_.empty()) {
        pending_route_rebuild_reason_ = reason;
    } else if (pending_route_rebuild_reason_.find(reason) == std::string::npos) {
        pending_route_rebuild_reason_ += "," + reason;
    }
}

bool LanePlannerNode::rebuild_route_from_pose(
    const Point2D& ego,
    const std::string& reason)
{
    const uint64_t start_lanelet_id = find_nearest_lanelet_from_pose(ego);
    if (start_lanelet_id == 0U) {
        throw std::runtime_error("failed to find nearest lanelet for route rebuild");
    }
    rebuild_route_from_lanelet(start_lanelet_id, reason);
    return true;
}

std::vector<uint64_t> LanePlannerNode::build_route_sequence_from_graph(
    const uint64_t start_lanelet_id,
    std::size_t& fallback_count) const
{
    if (lanelet_centerline_points_by_id_.find(start_lanelet_id) ==
        lanelet_centerline_points_by_id_.end())
    {
        throw std::runtime_error(
            "route start lanelet centerline points are not available: " +
            std::to_string(start_lanelet_id));
    }

    fallback_count = 0U;
    std::vector<uint64_t> route_lanelet_ids;
    route_lanelet_ids.reserve(static_cast<std::size_t>(route_lookahead_lanelet_count_));
    route_lanelet_ids.push_back(start_lanelet_id);

    while (route_lanelet_ids.size() < static_cast<std::size_t>(route_lookahead_lanelet_count_)) {
        uint8_t selected_turn = vectormap_msgs::msg::LaneConnection::TURN_UNKNOWN;
        bool used_fallback = false;
        const uint64_t next_lanelet_id = select_next_lanelet(
            route_lanelet_ids.back(),
            last_nav_cmd_turn_,
            selected_turn,
            used_fallback);
        if (next_lanelet_id == 0U || next_lanelet_id == start_lanelet_id) {
            break;
        }
        if (std::find(route_lanelet_ids.begin(), route_lanelet_ids.end(), next_lanelet_id) !=
            route_lanelet_ids.end())
        {
            RCLCPP_WARN(
                get_logger(),
                "route graph cycle detected at lanelet %lu; stop lookahead route generation",
                next_lanelet_id);
            break;
        }
        const auto next_centerline_it = lanelet_centerline_points_by_id_.find(next_lanelet_id);
        if (next_centerline_it == lanelet_centerline_points_by_id_.end()) {
            throw std::runtime_error(
                "selected lanelet centerline points are not available: " +
                std::to_string(next_lanelet_id));
        }
        route_lanelet_ids.push_back(next_lanelet_id);
        if (used_fallback) {
            ++fallback_count;
        }
    }

    return route_lanelet_ids;
}

uint64_t LanePlannerNode::select_next_lanelet(
    const uint64_t from_lanelet_id,
    const uint8_t requested_turn,
    uint8_t& selected_turn,
    bool& used_fallback) const
{
    const auto edges_it = connection_edges_by_from_lanelet_id_.find(from_lanelet_id);
    if (edges_it == connection_edges_by_from_lanelet_id_.end() || edges_it->second.empty()) {
        return 0U;
    }

    const auto find_min_cost_edge = [&edges = edges_it->second](const uint8_t turn_direction) {
        auto best_it = edges.end();
        for (auto it = edges.begin(); it != edges.end(); ++it) {
            if (it->turn_direction != turn_direction) {
                continue;
            }
            if (best_it == edges.end() || it->cost < best_it->cost) {
                best_it = it;
            }
        }
        return best_it;
    };

    const auto requested_it = find_min_cost_edge(requested_turn);
    if (requested_it != edges_it->second.end() && requested_it->turn_direction == requested_turn) {
        selected_turn = requested_turn;
        used_fallback = false;
        return requested_it->to_lanelet_id;
    }

    for (const auto fallback_turn : nav_cmd_fallback_order_) {
        if (fallback_turn == requested_turn) {
            continue;
        }
        const auto fallback_it = find_min_cost_edge(fallback_turn);
        if (fallback_it != edges_it->second.end() &&
            fallback_it->turn_direction == fallback_turn)
        {
            selected_turn = fallback_turn;
            used_fallback = true;
            return fallback_it->to_lanelet_id;
        }
    }

    return 0U;
}

uint8_t LanePlannerNode::parse_nav_cmd(const std::string& command) const
{
    if (command == "straight") {
        return vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT;
    }
    if (command == "left") {
        return vectormap_msgs::msg::LaneConnection::TURN_LEFT;
    }
    if (command == "right") {
        return vectormap_msgs::msg::LaneConnection::TURN_RIGHT;
    }
    throw std::invalid_argument("nav_cmd must be straight, left, or right: " + command);
}

std::string LanePlannerNode::turn_direction_to_string(
    const uint8_t turn_direction) const
{
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT) {
        return "straight";
    }
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_LEFT) {
        return "left";
    }
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_RIGHT) {
        return "right";
    }
    return "unknown";
}

uint64_t LanePlannerNode::find_nearest_lanelet_from_pose(
    const Point2D& point) const
{
    uint64_t best_lanelet_id = 0U;
    double best_distance_sq = std::numeric_limits<double>::max();
    for (const auto& [lanelet_id, centerline_points] : lanelet_centerline_points_by_id_) {
        if (centerline_points.size() < 2U) {
            continue;
        }
        for (std::size_t i = 1U; i < centerline_points.size(); ++i) {
            const double distance_sq = point_segment_distance_sq(
                point,
                centerline_points[i - 1U],
                centerline_points[i]);
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best_lanelet_id = lanelet_id;
            }
        }
    }
    return best_lanelet_id;
}

std::pair<uint64_t, double> LanePlannerNode::find_nearest_lanelet_within_route(
    const Point2D& point) const
{
    uint64_t best_lanelet_id = 0U;
    double best_distance_sq = std::numeric_limits<double>::max();
    for (const uint64_t lanelet_id : current_route_lanelet_ids_) {
        const auto centerline_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (centerline_it == lanelet_centerline_points_by_id_.end() ||
            centerline_it->second.size() < 2U)
        {
            continue;
        }
        const auto& centerline_points = centerline_it->second;
        for (std::size_t i = 1U; i < centerline_points.size(); ++i) {
            const double distance_sq = point_segment_distance_sq(
                point,
                centerline_points[i - 1U],
                centerline_points[i]);
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best_lanelet_id = lanelet_id;
            }
        }
    }
    const double distance = best_lanelet_id != 0U
        ? std::sqrt(best_distance_sq)
        : std::numeric_limits<double>::max();
    return {best_lanelet_id, distance};
}

uint64_t LanePlannerNode::lanelet_at_s(const double s) const
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

double LanePlannerNode::normalize_path_s(const double s) const
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

}  // namespace lane_planner
