#include "trajectory_follower/mpc/speed_limit.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{

namespace
{
constexpr double EPSILON = 1.0e-9;
}

double menger_curvature(
    const std::array<double, 2> & p0,
    const std::array<double, 2> & p1,
    const std::array<double, 2> & p2)
{
    const double area2 =
        (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    const double a = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
    const double b = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const double c = std::hypot(p2[0] - p0[0], p2[1] - p0[1]);
    const double denom = a * b * c;
    if (denom < EPSILON) {
        return 0.0;
    }
    return 2.0 * area2 / denom;
}

double forward_max_curvature(
    const std::vector<std::array<double, 2>> & path_xy,
    const std::vector<double> & arc,
    int nearest,
    double window)
{
    const int n = static_cast<int>(path_xy.size());
    double curvature = 0.0;
    for (int i = std::max(1, nearest); i < n - 1; ++i) {
        if (arc[i] - arc[nearest] > window) break;
        curvature = std::max(
            curvature, std::abs(menger_curvature(path_xy[i - 1], path_xy[i], path_xy[i + 1])));
    }
    return curvature;
}

double v_limit(
    double v_max,
    double a_lat_max,
    double a_min_abs,
    double kappa,
    double dist_to_end)
{
    double v = v_max;

    const double k = std::abs(kappa);
    if (k > 1.0e-6) {
        v = std::min(v, std::sqrt(a_lat_max / k));
    }

    const double v_stop =
        std::sqrt(std::max(0.0, 2.0 * std::abs(a_min_abs) * std::max(0.0, dist_to_end)));
    v = std::min(v, v_stop);

    return std::clamp(v, 0.0, v_max);
}

}
