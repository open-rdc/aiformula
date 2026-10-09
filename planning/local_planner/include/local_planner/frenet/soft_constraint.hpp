#pragma once

#include <vector>

namespace local_planner::frenet
{

struct CostWeights
{
    double curvature;
    double lateral_deviation;
    double side_change;
};

double curvature_cost(const std::vector<double> & curvatures, double weight);
double lateral_deviation_cost(double final_lateral_deviation, double weight);
double path_change_cost(double mean_deviation, double weight);
double candidate_cost(
    const std::vector<double> & curvatures, double final_lateral_deviation,
    double mean_deviation, const CostWeights & weights);

}
