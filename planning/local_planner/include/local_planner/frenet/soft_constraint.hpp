#pragma once

#include <vector>

namespace local_planner::frenet
{

struct CostWeights
{
    double curvature;
    double length;
    double lateral_deviation;
};

double curvature_cost(const std::vector<double> & curvatures, double weight);
double length_cost(double path_length, double weight);
double lateral_deviation_cost(double final_lateral_deviation, double weight);
double candidate_cost(
    const std::vector<double> & curvatures, double path_length,
    double final_lateral_deviation, const CostWeights & weights);

}
