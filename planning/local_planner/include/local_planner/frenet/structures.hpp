#pragma once

#include <optional>
#include <vector>

#include "local_planner/frenet/polynomial.hpp"

namespace local_planner::frenet
{

struct FrenetState
{
    double s;
    double d;
    double lateral_velocity;
    double lateral_acceleration;
};

struct PathCandidate
{
    std::vector<double> s_points;
    std::vector<double> offsets;
    std::optional<Polynomial> lateral_polynomial;
};

}
