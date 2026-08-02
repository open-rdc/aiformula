#pragma once

#include <vector>

#include "mission_planner/mission_planner_node.hpp"

namespace mission_planner
{

using Point2D = MissionPlannerNode::Point2D;

void append_centerline_points(
    std::vector<Point2D>& route_points,
    const std::vector<Point2D>& centerline_points,
    bool connects_to_previous,
    double min_gap_m);

}
