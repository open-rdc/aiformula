#pragma once

namespace trajectory_follower
{

struct LongitudinalParams
{
    double v_max;
    double a_max;
    double a_min;
    double max_integral_effort;
    double jerk_max;
    double kp;
    double ki;
    double kd;
    double lpf_vel_error_gain;
    double dt;
};

class LongitudinalPid
{
public:
    void configure(const LongitudinalParams & params);

    double update(double v_ref, double a_ref, double v_meas);

    void reset();

private:
    LongitudinalParams params_{};
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double filtered_error_ = 0.0;
    double prev_a_cmd_ = 0.0;
    double v_cmd_ = 0.0;
    bool initialized_ = false;
};

}
