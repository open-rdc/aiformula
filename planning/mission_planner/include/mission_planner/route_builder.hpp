#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "mission_planner/mission_planner_node.hpp"

namespace mission_planner
{

using Point2D = MissionPlannerNode::Point2D;
using RouteEdge = MissionPlannerNode::RouteEdge;

void append_centerline_points(
    std::vector<Point2D>& route_points,
    const std::vector<Point2D>& centerline_points,
    bool connects_to_previous,
    double min_gap_m);

double menger_curvature(const Point2D& a, const Point2D& b, const Point2D& c);

std::vector<Point2D> catmull_rom_smooth(
    const std::vector<Point2D>& control_points,
    int samples_per_segment);

uint64_t select_start_lanelet(
    const std::unordered_map<uint64_t, std::vector<Point2D>>& centerlines,
    const Point2D& point,
    double yaw,
    double yaw_threshold_rad,
    double max_distance_m);

bool has_connection(
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    uint64_t from_lanelet_id,
    uint64_t to_lanelet_id);

uint64_t select_next_lanelet_id(
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    uint64_t from_lanelet_id,
    uint8_t requested_turn,
    const std::vector<uint8_t>& fallback_order,
    bool& used_fallback);

std::vector<uint64_t> find_route_lanelet_ids(
    const std::unordered_map<uint64_t, std::vector<Point2D>>& centerlines,
    const std::unordered_map<uint64_t, std::vector<RouteEdge>>& edges_by_from_lanelet_id,
    const std::vector<uint8_t>& fallback_order,
    uint64_t start_lanelet_id,
    uint8_t requested_turn,
    int lookahead_count,
    std::size_t& fallback_count);

}
