#pragma once

#include <array>
#include <vector>

namespace trajectory_follower
{

struct LateralMpcParams
{
    double wheelbase = 0.8;
    double steer_tau = 0.3;
    double steer_limit = 0.2618;
    double weight_lat_error = 1.0;
    double weight_heading_error = 1.0;
    double weight_steering_input = 0.5;
    double weight_steer_rate = 5.0;
    double weight_terminal_lat_error = 1.0;
    double weight_terminal_heading_error = 1.0;
    int horizon = 20;
    double prediction_dt = 0.1;
    double min_predict_speed = 0.5;
};

class LateralMpc
{
public:
    void configure(const LateralMpcParams & params);

    double computeSteering(const std::vector<std::array<double, 2>> & path_xy, double v);

    void reset();

    double previousSteer() const { return prev_steer_; }

private:
    LateralMpcParams p_{};
    double prev_steer_ = 0.0;
};

}
