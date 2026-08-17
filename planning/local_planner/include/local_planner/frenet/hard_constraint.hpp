#pragma once

#include <vector>

namespace local_planner::frenet
{

bool is_collision_free(
    const std::vector<double> & s_grid, const std::vector<double> & offsets,
    double obstacle_s, double obstacle_d,
    double lateral_margin, double longitudinal_margin);

bool satisfies_curvature_limit(
    const std::vector<double> & curvatures, double kappa_max);

}
