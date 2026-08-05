#include "mission_planner/mission_planner_node.hpp"
#include "mission_planner/route_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include <vectormap_msgs/msg/lane_connection.hpp>
#include <vectormap_msgs/msg/line_string.hpp>

namespace mission_planner
{
namespace
{
using Point2D = MissionPlannerNode::Point2D;
using PathPoint = MissionPlannerNode::PathPoint;
using RouteEdge = MissionPlannerNode::RouteEdge;
using LaneConnection = vectormap_msgs::msg::LaneConnection;
using LineString = vectormap_msgs::msg::LineString;

constexpr double EPSILON = 1.0e-6;
constexpr int kSmoothingSamplesPerSegment = 10;

double distance_2d(const Point2D& a, const Point2D& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double point_segment_distance_sq(
    const Point2D& point,
    const Point2D& start,
    const Point2D& end)
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

PathPoint interpolate_raw_path(
    const std::vector<Point2D>& points,
    const std::vector<double>& s_values,
    const double target_s)
{
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
    return PathPoint{x, y, yaw};
}

Point2D lerp_point(const Point2D& a, const Point2D& b, const double t)
{
    return Point2D{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

Point2D blend_point(
    const Point2D& a,
    const Point2D& b,
    const double ta,
    const double tb,
    const double t)
{
    const double denom = tb - ta;
    const double ratio = denom > EPSILON ? (t - ta) / denom : 0.0;
    return lerp_point(a, b, ratio);
}

Point2D catmull_rom_point(
    const Point2D& p0,
    const Point2D& p1,
    const Point2D& p2,
    const Point2D& p3,
    const double u)
{
    constexpr double kCentripetalAlpha = 0.5;
    const double t0 = 0.0;
    const double t1 = t0 + std::max(std::pow(distance_2d(p0, p1), kCentripetalAlpha), EPSILON);
    const double t2 = t1 + std::max(std::pow(distance_2d(p1, p2), kCentripetalAlpha), EPSILON);
    const double t3 = t2 + std::max(std::pow(distance_2d(p2, p3), kCentripetalAlpha), EPSILON);
    const double t = t1 + u * (t2 - t1);

    const auto a1 = blend_point(p0, p1, t0, t1, t);
    const auto a2 = blend_point(p1, p2, t1, t2, t);
    const auto a3 = blend_point(p2, p3, t2, t3, t);
    const auto b1 = blend_point(a1, a2, t0, t2, t);
    const auto b2 = blend_point(a2, a3, t1, t3, t);
    return blend_point(b1, b2, t1, t2, t);
}

template <typename LaneletIdContainer>
std::pair<uint64_t, double> nearest_lanelet_in(
    const std::unordered_map<uint64_t, std::vector<Point2D>>& centerlines,
    const LaneletIdContainer& lanelet_ids,
    const Point2D& point)
{
    uint64_t best_lanelet_id = 0U;
    double best_distance_sq = std::numeric_limits<double>::max();
    for (const uint64_t lanelet_id : lanelet_ids) {
        const auto centerline_it = centerlines.find(lanelet_id);
        if (centerline_it == centerlines.end() || centerline_it->second.size() < 2U) {
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
    return {best_lanelet_id, best_distance_sq};
}

std::string turn_direction_to_string(const uint8_t turn_direction)
{
    if (turn_direction == LaneConnection::TURN_STRAIGHT) {
        return "straight";
    }
    if (turn_direction == LaneConnection::TURN_LEFT) {
        return "left";
    }
    if (turn_direction == LaneConnection::TURN_RIGHT) {
        return "right";
    }
    return "unknown";
}

}

double menger_curvature(const Point2D& a, const Point2D& b, const Point2D& c)
{
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double bcx = c.x - b.x;
    const double bcy = c.y - b.y;
    const double cross = abx * bcy - aby * bcx;
    const double length_ab = std::hypot(abx, aby);
    const double length_bc = std::hypot(bcx, bcy);
    const double length_ac = distance_2d(a, c);
    const double denom = length_ab * length_bc * length_ac;
    if (denom <= EPSILON) {
        return 0.0;
    }
    return 2.0 * std::abs(cross) / denom;
}

std::vector<Point2D> catmull_rom_smooth(
    const std::vector<Point2D>& control_points,
    const int samples_per_segment)
{
    if (control_points.size() < 2U || samples_per_segment < 1) {
        return control_points;
    }

    std::vector<Point2D> padded;
    padded.reserve(control_points.size() + 2U);
    padded.push_back(control_points.front());
    padded.insert(padded.end(), control_points.begin(), control_points.end());
    padded.push_back(control_points.back());

    std::vector<Point2D> smoothed;
    smoothed.reserve(control_points.size() * static_cast<std::size_t>(samples_per_segment) + 1U);
    const std::size_t segment_count = control_points.size() - 1U;
    for (std::size_t segment = 0U; segment < segment_count; ++segment) {
        const auto& p0 = padded[segment];
        const auto& p1 = padded[segment + 1U];
        const auto& p2 = padded[segment + 2U];
        const auto& p3 = padded[segment + 3U];
        for (int sample = 0; sample < samples_per_segment; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(samples_per_segment);
            smoothed.push_back(catmull_rom_point(p0, p1, p2, p3, u));
        }
    }
    smoothed.push_back(control_points.back());
    return smoothed;
}

uint64_t select_start_lanelet(
    const std::unordered_map<uint64_t, std::vector<Point2D>>& centerlines,
    const Point2D& point,
    const double yaw,
    const double yaw_threshold_rad,
    const double max_distance_m)
{
    uint64_t best_lanelet_id = 0U;
    double best_distance_sq = std::numeric_limits<double>::max();
    const double max_distance_sq = max_distance_m * max_distance_m;

    for (const auto& [lanelet_id, centerline_points] : centerlines) {
        if (centerline_points.size() < 2U) {
            continue;
        }

        double nearest_distance_sq = std::numeric_limits<double>::max();
        std::size_t nearest_segment_end = 1U;
        for (std::size_t i = 1U; i < centerline_points.size(); ++i) {
            const double distance_sq = point_segment_distance_sq(
                point, centerline_points[i - 1U], centerline_points[i]);
            if (distance_sq < nearest_distance_sq) {
                nearest_distance_sq = distance_sq;
                nearest_segment_end = i;
            }
        }

        if (nearest_distance_sq > max_distance_sq || nearest_distance_sq >= best_distance_sq) {
            continue;
        }

        const auto& start = centerline_points[nearest_segment_end - 1U];
        const auto& end = centerline_points[nearest_segment_end];
        const double segment_yaw = std::atan2(end.y - start.y, end.x - start.x);
        const double yaw_difference = std::atan2(
            std::sin(yaw - segment_yaw),
            std::cos(yaw - segment_yaw));
        if (std::abs(yaw_difference) > yaw_threshold_rad) {
            continue;
        }

        best_distance_sq = nearest_distance_sq;
        best_lanelet_id = lanelet_id;
    }

    return best_lanelet_id;
}

bool has_connection(
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    const uint64_t from_lanelet_id,
    const uint64_t to_lanelet_id)
{
    const auto edges_it = edges_by_from_lanelet_id.find(from_lanelet_id);
    if (edges_it == edges_by_from_lanelet_id.end()) {
        return false;
    }
    return std::any_of(
        edges_it->second.begin(),
        edges_it->second.end(),
        [to_lanelet_id](const RouteEdge& edge) {
            return edge.to_lanelet_id == to_lanelet_id;
        });
}

uint64_t select_next_lanelet_id(
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    const uint64_t from_lanelet_id,
    const uint8_t requested_turn,
    const std::vector<uint8_t>& fallback_order,
    bool& used_fallback)
{
    const auto edges_it = edges_by_from_lanelet_id.find(from_lanelet_id);
    if (edges_it == edges_by_from_lanelet_id.end()) {
        return 0U;
    }

    const auto find_min_cost_edge =
        [&edges = edges_it->second](const uint8_t turn_direction) -> const RouteEdge* {
            const RouteEdge* best = nullptr;
            for (const auto& edge : edges) {
                if (edge.turn_direction == turn_direction &&
                    (best == nullptr || edge.cost < best->cost))
                {
                    best = &edge;
                }
            }
            return best;
        };

    if (const RouteEdge* requested_edge = find_min_cost_edge(requested_turn)) {
        used_fallback = false;
        return requested_edge->to_lanelet_id;
    }

    for (const auto fallback_turn : fallback_order) {
        if (fallback_turn == requested_turn) {
            continue;
        }
        if (const RouteEdge* fallback_edge = find_min_cost_edge(fallback_turn)) {
            used_fallback = true;
            return fallback_edge->to_lanelet_id;
        }
    }

    return 0U;
}

std::vector<uint64_t> find_route_lanelet_ids(
    const std::unordered_map<uint64_t, std::vector<Point2D>>& centerlines,
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    const std::vector<uint8_t>& fallback_order,
    const uint64_t start_lanelet_id,
    const uint8_t requested_turn,
    const int lookahead_count,
    std::size_t& fallback_count)
{
    fallback_count = 0U;
    std::vector<uint64_t> route_lanelet_ids;
    route_lanelet_ids.reserve(static_cast<std::size_t>(lookahead_count));
    route_lanelet_ids.push_back(start_lanelet_id);

    while (route_lanelet_ids.size() < static_cast<std::size_t>(lookahead_count)) {
        bool used_fallback = false;
        const uint64_t next_lanelet_id = select_next_lanelet_id(
            edges_by_from_lanelet_id, route_lanelet_ids.back(), requested_turn, fallback_order, used_fallback);
        if (next_lanelet_id == 0U || next_lanelet_id == start_lanelet_id) {
            break;
        }
        if (std::find(route_lanelet_ids.begin(), route_lanelet_ids.end(), next_lanelet_id) !=
            route_lanelet_ids.end())
        {
            break;
        }
        if (centerlines.find(next_lanelet_id) == centerlines.end()) {
            break;
        }
        route_lanelet_ids.push_back(next_lanelet_id);
        if (used_fallback) {
            ++fallback_count;
        }
    }

    return route_lanelet_ids;
}

void append_centerline_points(
    std::vector<Point2D>& route_points,
    const std::vector<Point2D>& centerline_points,
    const bool connects_to_previous,
    const double min_gap_m)
{
    for (std::size_t point_index = 0U; point_index < centerline_points.size(); ++point_index) {
        if (connects_to_previous && point_index == 0U) {
            continue;
        }
        const auto& point = centerline_points[point_index];
        if (!route_points.empty() && distance_2d(route_points.back(), point) <= min_gap_m) {
            continue;
        }
        route_points.push_back(point);
    }
}

void MissionPlannerNode::build_map_lookup(
    const vectormap_msgs::msg::VectorMap& map_msg)
{
    if (map_msg.header.frame_id != "map") {
        RCLCPP_WARN(
            get_logger(),
            "VectorMapのframe_idはmapである必要があります（実際: %s）: マップ構築を中止します",
            map_msg.header.frame_id.c_str());
        return;
    }

    std::unordered_map<uint64_t, const LineString*> line_string_by_id;
    line_string_by_id.reserve(map_msg.line_strings.size());
    for (const auto& line_string : map_msg.line_strings) {
        line_string_by_id.emplace(line_string.id, &line_string);
    }

    lanelet_centerline_points_by_id_.clear();
    lanelet_centerline_points_by_id_.reserve(map_msg.lanelets.size());
    for (const auto& lanelet : map_msg.lanelets) {
        const auto line_it = line_string_by_id.find(lanelet.centerline_id);
        if (line_it == line_string_by_id.end() || line_it->second->points.size() < 2U) {
            continue;
        }
        const LineString& centerline = *line_it->second;
        if (centerline.line_type != LineString::TYPE_VIRTUAL_LINE ||
            centerline.marking_type != LineString::MARKING_VIRTUAL)
        {
            RCLCPP_WARN(
                get_logger(),
                "centerlineはvirtual lineである必要があります（lanelet_id=%lu）: マップ構築を中止します",
                lanelet.id);
            return;
        }
        std::vector<Point2D> centerline_points;
        centerline_points.reserve(centerline.points.size());
        for (const auto& point : centerline.points) {
            centerline_points.push_back(Point2D{point.x, point.y});
        }
        lanelet_centerline_points_by_id_.emplace(lanelet.id, std::move(centerline_points));
    }

    connection_edges_by_from_lanelet_id_.clear();
    connection_edges_by_from_lanelet_id_.reserve(map_msg.lane_connections.size());
    for (const auto& connection : map_msg.lane_connections) {
        connection_edges_by_from_lanelet_id_[connection.from_lanelet_id].push_back(
            RouteEdge{connection.to_lanelet_id, connection.turn_direction, connection.cost});
    }

    std::unordered_map<uint64_t, uint64_t> lanelet_by_left_line;
    std::unordered_map<uint64_t, uint64_t> lanelet_by_right_line;
    lanelet_by_left_line.reserve(map_msg.lanelets.size());
    lanelet_by_right_line.reserve(map_msg.lanelets.size());
    for (const auto& lanelet : map_msg.lanelets) {
        lanelet_by_left_line[lanelet.left_line_id] = lanelet.id;
        lanelet_by_right_line[lanelet.right_line_id] = lanelet.id;
    }
    left_adjacent_lanelet_by_id_.clear();
    right_adjacent_lanelet_by_id_.clear();
    for (const auto& lanelet : map_msg.lanelets) {
        const auto left_it = lanelet_by_right_line.find(lanelet.left_line_id);
        if (left_it != lanelet_by_right_line.end() && left_it->second != lanelet.id) {
            left_adjacent_lanelet_by_id_[lanelet.id] = left_it->second;
        }
        const auto right_it = lanelet_by_left_line.find(lanelet.right_line_id);
        if (right_it != lanelet_by_left_line.end() && right_it->second != lanelet.id) {
            right_adjacent_lanelet_by_id_[lanelet.id] = right_it->second;
        }
    }

    map_ready_ = true;
    RCLCPP_INFO(
        get_logger(),
        "built vector map lookup: lanelets=%zu, centerlines=%zu",
        map_msg.lanelets.size(),
        lanelet_centerline_points_by_id_.size());
}

bool MissionPlannerNode::try_start_initial_route(const Point2D& ego, const double yaw)
{
    const uint64_t start_lanelet_id = select_start_lanelet(
        lanelet_centerline_points_by_id_,
        ego,
        yaw,
        start_lanelet_yaw_threshold_rad_,
        start_lanelet_max_distance_m_);
    if (start_lanelet_id == 0U) {
        return false;
    }
    replan_route_from_lanelet(start_lanelet_id, "initial");
    if (current_route_lanelet_ids_.empty()) {
        return false;
    }
    global_path_ready_ = true;
    return true;
}

bool MissionPlannerNode::apply_route_lanelet_ids(
    const std::vector<uint64_t>& route_lanelet_ids)
{
    if (route_lanelet_ids.empty()) {
        RCLCPP_ERROR(get_logger(), "ルートのlanelet列が空のため構築できません");
        return false;
    }

    std::vector<Point2D> route_points;
    route_points.reserve(route_lanelet_ids.size() * 16U);

    for (std::size_t route_index = 0U; route_index < route_lanelet_ids.size(); ++route_index) {
        const uint64_t lanelet_id = route_lanelet_ids[route_index];
        if (route_index + 1U < route_lanelet_ids.size()) {
            const uint64_t next_id = route_lanelet_ids[route_index + 1U];
            if (!has_connection(connection_edges_by_from_lanelet_id_, lanelet_id, next_id)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "LaneConnectionが存在しません: %lu -> %lu",
                    lanelet_id, next_id);
                return false;
            }
        }

        const auto centerline_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (centerline_it == lanelet_centerline_points_by_id_.end()) {
            RCLCPP_ERROR(
                get_logger(), "ルートのcenterline点群が取得できません: %lu", lanelet_id);
            return false;
        }
        const auto& centerline_points = centerline_it->second;
        const bool connects_to_previous = !route_points.empty();
        if (connects_to_previous) {
            const double gap = distance_2d(route_points.back(), centerline_points.front());
            if (gap > max_centerline_connection_gap_m_) {
                RCLCPP_ERROR(
                    get_logger(),
                    "centerlineの接続ギャップが大きすぎます: %f m (lanelet_id=%lu)",
                    gap, lanelet_id);
                return false;
            }
        }

        append_centerline_points(
            route_points, centerline_points, connects_to_previous, max_centerline_connection_gap_m_);
    }

    if (route_points.size() < 2U) {
        RCLCPP_ERROR(get_logger(), "ルートのcenterline点数が2未満です");
        return false;
    }

    const bool route_is_loop = has_connection(
        connection_edges_by_from_lanelet_id_,
        route_lanelet_ids.back(),
        route_lanelet_ids.front());
    if (route_is_loop) {
        const double closing_gap = distance_2d(route_points.back(), route_points.front());
        if (closing_gap > max_centerline_connection_gap_m_) {
            RCLCPP_ERROR(
                get_logger(),
                "ループ閉合のcenterlineギャップが大きすぎます: %f m",
                closing_gap);
            return false;
        }
        if (closing_gap > EPSILON) {
            route_points.push_back(route_points.front());
        }
    }

    const auto smoothed_points = catmull_rom_smooth(route_points, kSmoothingSamplesPerSegment);

    std::vector<Point2D> resample_points;
    std::vector<double> raw_s;
    resample_points.reserve(smoothed_points.size());
    raw_s.reserve(smoothed_points.size());
    double accumulated_s = 0.0;
    for (const auto& point : smoothed_points) {
        if (!resample_points.empty()) {
            const double ds = distance_2d(resample_points.back(), point);
            if (ds <= EPSILON) {
                continue;
            }
            accumulated_s += ds;
        }
        resample_points.push_back(point);
        raw_s.push_back(accumulated_s);
    }

    if (resample_points.size() < 2U) {
        RCLCPP_ERROR(get_logger(), "平滑化後のルート点数が2未満です");
        return false;
    }

    global_samples_.clear();
    global_samples_.reserve(
        static_cast<std::size_t>(std::ceil(raw_s.back() / global_path_resample_interval_m_)) + 2U);
    for (double s = 0.0; s < raw_s.back(); s += global_path_resample_interval_m_) {
        global_samples_.push_back(interpolate_raw_path(resample_points, raw_s, s));
    }
    global_samples_.push_back(interpolate_raw_path(resample_points, raw_s, raw_s.back()));
    current_route_lanelet_ids_ = route_lanelet_ids;
    current_route_is_loop_ = route_is_loop;
    report_curvature_qa();
    return true;
}

void MissionPlannerNode::replan_route_from_lanelet(
    const uint64_t start_lanelet_id,
    const std::string& reason)
{
    std::size_t fallback_count = 0U;
    const auto route_lanelet_ids = search_route_lanelet_ids(start_lanelet_id, fallback_count);
    if (route_lanelet_ids.empty() || !apply_route_lanelet_ids(route_lanelet_ids)) {
        RCLCPP_ERROR(
            get_logger(),
            "ルート再構築に失敗したため既存ルートを維持します: reason=%s start_lanelet=%lu",
            reason.c_str(),
            start_lanelet_id);
        return;
    }
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
            "route rebuild used navigation_command fallback %zu times: requested=%s",
            fallback_count,
            turn_direction_to_string(last_navigation_command_turn_).c_str());
    }
}

bool MissionPlannerNode::replan_route_from_pose(
    const Point2D& ego,
    const double yaw,
    const std::string& reason)
{
    const auto reachable = build_reachable_lanelet_set();
    uint64_t start_lanelet_id =
        nearest_lanelet_in(lanelet_centerline_points_by_id_, reachable, ego).first;
    if (start_lanelet_id == 0U) {
        RCLCPP_WARN(
            get_logger(),
            "route rebuild skipped: no reachable lanelet near ego pose "
            "(reason=%s, navigation_command=%s) — connection constraints not satisfied. "
            "全laneletからのフォールバック探索を試行します",
            reason.c_str(),
            turn_direction_to_string(last_navigation_command_turn_).c_str());
        start_lanelet_id = select_start_lanelet(
            lanelet_centerline_points_by_id_,
            ego,
            yaw,
            start_lanelet_yaw_threshold_rad_,
            start_lanelet_max_distance_m_);
        if (start_lanelet_id == 0U) {
            RCLCPP_ERROR(
                get_logger(),
                "フォールバック探索でも起点laneletが見つかりませんでした: reason=%s",
                reason.c_str());
            return false;
        }
        replan_route_from_lanelet(start_lanelet_id, "out_of_route_fallback");
        return true;
    }
    replan_route_from_lanelet(start_lanelet_id, reason);
    return true;
}

std::vector<uint64_t> MissionPlannerNode::search_route_lanelet_ids(
    const uint64_t start_lanelet_id,
    std::size_t& fallback_count) const
{
    if (lanelet_centerline_points_by_id_.find(start_lanelet_id) ==
        lanelet_centerline_points_by_id_.end())
    {
        RCLCPP_ERROR(
            get_logger(),
            "起点laneletのcenterlineが見つからないためルート探索を中止します: %lu",
            start_lanelet_id);
        fallback_count = 0U;
        return {};
    }
    return find_route_lanelet_ids(
        lanelet_centerline_points_by_id_,
        connection_edges_by_from_lanelet_id_,
        navigation_command_fallback_order_,
        start_lanelet_id,
        last_navigation_command_turn_,
        route_lookahead_lanelet_count_,
        fallback_count);
}

uint64_t MissionPlannerNode::select_next_lanelet(
    const uint64_t from_lanelet_id,
    const uint8_t requested_turn,
    bool& used_fallback) const
{
    return select_next_lanelet_id(
        connection_edges_by_from_lanelet_id_, from_lanelet_id, requested_turn,
        navigation_command_fallback_order_, used_fallback);
}

std::unordered_set<uint64_t> MissionPlannerNode::build_reachable_lanelet_set() const
{
    std::unordered_set<uint64_t> reachable(
        current_route_lanelet_ids_.begin(), current_route_lanelet_ids_.end());
    for (const uint64_t id : current_route_lanelet_ids_) {
        const auto edges_it = connection_edges_by_from_lanelet_id_.find(id);
        if (edges_it == connection_edges_by_from_lanelet_id_.end()) {
            continue;
        }
        for (const auto& edge : edges_it->second) {
            if (edge.turn_direction == last_navigation_command_turn_) {
                reachable.insert(edge.to_lanelet_id);
            }
        }
    }
    return reachable;
}

std::pair<uint64_t, double> MissionPlannerNode::find_nearest_lanelet_within_route(
    const Point2D& point) const
{
    const auto [lanelet_id, distance_sq] = nearest_lanelet_in(
        lanelet_centerline_points_by_id_, current_route_lanelet_ids_, point);
    const double distance = lanelet_id != 0U
        ? std::sqrt(distance_sq)
        : std::numeric_limits<double>::max();
    return {lanelet_id, distance};
}

void MissionPlannerNode::report_curvature_qa() const
{
    if (global_samples_.size() < 3U) {
        return;
    }

    double cumulative_s = 0.0;
    double max_curvature = 0.0;
    double max_curvature_s = 0.0;
    for (std::size_t i = 1U; i + 1U < global_samples_.size(); ++i) {
        cumulative_s += std::hypot(
            global_samples_[i].x - global_samples_[i - 1U].x,
            global_samples_[i].y - global_samples_[i - 1U].y);
        const Point2D a{global_samples_[i - 1U].x, global_samples_[i - 1U].y};
        const Point2D b{global_samples_[i].x, global_samples_[i].y};
        const Point2D c{global_samples_[i + 1U].x, global_samples_[i + 1U].y};
        const double curvature = menger_curvature(a, b, c);
        if (curvature > max_curvature) {
            max_curvature = curvature;
            max_curvature_s = cumulative_s;
        }
    }

    if (max_curvature > curvature_limit_per_m_) {
        RCLCPP_WARN(
            get_logger(),
            "生成した経路の曲率が上限を超過しています: kappa=%.3f 1/m (limit=%.3f 1/m) s=%.2f m",
            max_curvature,
            curvature_limit_per_m_,
            max_curvature_s);
    }
}

}
