#include "local_planner/frenet/frenet_planner.hpp"

#include <cmath>

namespace local_planner::frenet
{
namespace
{
constexpr double EPSILON = 1.0e-6;
}

PathCandidate generate_candidate(
    const FrenetState & initial, const FrenetState & target, const double s_resolution)
{
    PathCandidate candidate;
    const double delta_s = target.s - initial.s;
    if (delta_s < s_resolution || s_resolution <= EPSILON) {
        return candidate;
    }
    candidate.lateral_polynomial.emplace(
        initial.d, initial.lateral_velocity, initial.lateral_acceleration,
        target.d, target.lateral_velocity, target.lateral_acceleration, delta_s);
    const std::size_t count = static_cast<std::size_t>(delta_s / s_resolution) + 1U;
    candidate.s_points.reserve(count);
    candidate.offsets.reserve(count);
    for (double s = s_resolution; s <= delta_s + EPSILON; s += s_resolution) {
        candidate.s_points.push_back(initial.s + s);
        candidate.offsets.push_back(candidate.lateral_polynomial->position(s));
    }
    return candidate;
}

}
