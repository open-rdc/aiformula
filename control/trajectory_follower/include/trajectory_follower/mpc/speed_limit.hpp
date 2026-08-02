#pragma once

#include <array>
#include <vector>

namespace trajectory_follower
{

double menger_curvature(
    const std::array<double, 2> & p0,
    const std::array<double, 2> & p1,
    const std::array<double, 2> & p2);

double forward_max_curvature(
    const std::vector<std::array<double, 2>> & path_xy,
    const std::vector<double> & arc,
    int nearest,
    double window);

double v_limit(
    double v_max,
    double a_lat_max,
    double a_min_abs,
    double kappa,
    double dist_to_end);

}
