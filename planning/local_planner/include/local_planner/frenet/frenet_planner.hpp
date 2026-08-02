#pragma once

#include "local_planner/frenet/structures.hpp"

namespace local_planner::frenet
{

PathCandidate generate_candidate(
    const FrenetState & initial, const FrenetState & target, double s_resolution);

}
