#pragma once

#include <vector>

namespace local_planner::frenet
{

double lateral_clearance(
    const std::vector<double> & s_grid, const std::vector<double> & offsets,
    double obstacle_s, double obstacle_d, double obstacle_half_width,
    double longitudinal_margin);

bool satisfies_curvature_limit(
    const std::vector<double> & curvatures, double kappa_max);

}
