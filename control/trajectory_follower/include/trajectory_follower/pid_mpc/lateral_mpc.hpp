#pragma once

#include <array>
#include <vector>

namespace trajectory_follower
{

struct LateralMpcParams
{
    double wheelbase;
    double steer_tau;
    double steer_limit;
    double weight_lat_error;
    double weight_heading_error;
    double weight_steering_input;
    double weight_steer_rate;
    double weight_terminal_lat_error;
    double weight_terminal_heading_error;
    int horizon;
    double prediction_dt;
    double min_predict_speed;
    double control_dt;
    double offset_update_vel_threshold;
    double offset_update_steer_threshold;
    double offset_time_constant;
    double offset_limit;
};

struct TrackingError
{
    double e_y;
    double e_yaw;
    int nearest;
};

// path_xy は base_link 座標
TrackingError computeTrackingError(
    const std::vector<std::array<double, 2>> & path_xy,
    const std::vector<double> & curvatures);

class LateralMpc
{
public:
    void configure(const LateralMpcParams & params);

    double computeSteering(
        const std::vector<std::array<double, 2>> & path_xy,
        const std::vector<double> & curvatures,
        double v);

    void reset();

    void setMeasuredSteer(double steer);

    void setYawRate(double yaw_rate);

    double steeringOffset() const { return steering_offset_; }

    double previousSteer() const { return prev_steer_; }

private:
    LateralMpcParams params_{};
    double prev_steer_ = 0.0;
    double steer_state_ = 0.0;
    double yaw_rate_ = 0.0;
    bool has_yaw_rate_ = false;
    double steering_offset_ = 0.0;
};

}
