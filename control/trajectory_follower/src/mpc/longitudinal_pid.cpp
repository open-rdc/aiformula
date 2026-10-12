#include "trajectory_follower/mpc/longitudinal_pid.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{

namespace
{
constexpr double STOP_VELOCITY_EPS = 0.05;
}

void LongitudinalPid::configure(const LongitudinalParams & params)
{
    params_ = params;
    reset();
}

double LongitudinalPid::update(double v_ref, double a_ref, double v_meas)
{
    if (v_ref <= STOP_VELOCITY_EPS && std::abs(v_meas) <= STOP_VELOCITY_EPS) {
        integral_ = 0.0;
        prev_error_ = 0.0;
        filtered_error_ = 0.0;
        prev_a_cmd_ = 0.0;
        v_cmd_ = 0.0;
        initialized_ = true;
        return 0.0;
    }

    const double dt = params_.dt;
    const double error = v_ref - v_meas;

    if (!initialized_) {
        v_cmd_ = std::clamp(v_meas, 0.0, params_.v_max);
        filtered_error_ = error;
        prev_error_ = error;
    }

    const double g = std::clamp(params_.lpf_vel_error_gain, 0.0, 1.0);
    filtered_error_ = g * filtered_error_ + (1.0 - g) * error;

    const bool saturated =
        (v_cmd_ >= params_.v_max && filtered_error_ > 0.0) || (v_cmd_ <= 0.0 && filtered_error_ < 0.0);
    if (!saturated) {
        integral_ += filtered_error_ * dt;
    }
    if (params_.ki > 1.0e-9) {
        const double i_limit = params_.max_integral_effort / params_.ki;
        integral_ = std::clamp(integral_, -i_limit, i_limit);
    }
    const double deriv = initialized_ ? (filtered_error_ - prev_error_) / dt : 0.0;
    const double a_fb = params_.kp * filtered_error_ + params_.ki * integral_ + params_.kd * deriv;

    double a_cmd = std::clamp(a_ref + a_fb, params_.a_min, params_.a_max);
    const double da_max = params_.jerk_max * dt;
    a_cmd = std::clamp(a_cmd, prev_a_cmd_ - da_max, prev_a_cmd_ + da_max);

    v_cmd_ = std::clamp(v_cmd_ + a_cmd * dt, 0.0, params_.v_max);

    prev_error_ = filtered_error_;
    prev_a_cmd_ = a_cmd;
    initialized_ = true;
    return v_cmd_;
}

void LongitudinalPid::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_error_ = 0.0;
    prev_a_cmd_ = 0.0;
    v_cmd_ = 0.0;
    initialized_ = false;
}

}
