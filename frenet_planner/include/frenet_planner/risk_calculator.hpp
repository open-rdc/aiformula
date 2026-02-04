#pragma once

#include <vector>
#include <cmath>
#include "frenet_planner/obstacle_detect.hpp"

namespace frenet_planner {

struct TrajectoryPoint;
struct FrenetTrajectory;

class RiskCalculator {
public:
    RiskCalculator();

    void set_parameters(
        double k_jerk,
        double k_time,
        double k_d,
        double k_s_dot,
        double k_lat,
        double k_lon,
        double target_speed
    );

    double calculate_cost(
        const FrenetTrajectory& trajectory,
        const std::vector<Obstacle>& obstacles
    );

private:
    double k_jerk_;
    double k_time_;
    double k_d_;
    double k_s_dot_;
    double k_lateral_ = 1.0;
    double k_longitudinal_ = 1.0;
    double target_speed_;
};

}  // namespace frenet_planner
