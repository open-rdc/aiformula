#include "local_planner/frenet/soft_constraint.hpp"

#include <cmath>

namespace local_planner::frenet
{

double curvature_cost(const std::vector<double> & curvatures, const double weight)
{
    if (curvatures.empty()) {
        return 0.0;
    }
    double curvature_sum = 0.0;
    for (const double curvature : curvatures) {
        curvature_sum += std::abs(curvature);
    }
    return weight * curvature_sum / static_cast<double>(curvatures.size());
}

double lateral_deviation_cost(const double final_lateral_deviation, const double weight)
{
    return weight * std::abs(final_lateral_deviation);
}

double path_change_cost(const double mean_deviation, const double weight)
{
    return weight * mean_deviation;
}

double candidate_cost(
    const std::vector<double> & curvatures, const double final_lateral_deviation,
    const double mean_deviation, const CostWeights & weights)
{
    return curvature_cost(curvatures, weights.curvature) +
        lateral_deviation_cost(final_lateral_deviation, weights.lateral_deviation) +
        path_change_cost(mean_deviation, weights.side_change);
}

}
