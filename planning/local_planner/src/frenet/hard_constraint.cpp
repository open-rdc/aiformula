#include "local_planner/frenet/hard_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner::frenet
{

double lateral_clearance(
    const std::vector<double> & s_grid, const std::vector<double> & offsets,
    const double obstacle_s, const double obstacle_d, const double obstacle_half_width,
    const double longitudinal_margin)
{
    double clearance = std::numeric_limits<double>::infinity();
    const std::size_t size = std::min(s_grid.size(), offsets.size());
    for (std::size_t i = 0U; i < size; ++i) {
        if (std::abs(s_grid[i] - obstacle_s) > longitudinal_margin) {
            continue;
        }
        clearance = std::min(
            clearance, std::abs(offsets[i] - obstacle_d) - obstacle_half_width);
    }
    return clearance;
}

bool satisfies_curvature_limit(
    const std::vector<double> & curvatures, const double kappa_max)
{
    for (const double curvature : curvatures) {
        if (std::abs(curvature) > kappa_max) {
            return false;
        }
    }
    return true;
}

}
