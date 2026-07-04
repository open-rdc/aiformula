#pragma once

namespace trajectory_follower
{

struct LongitudinalParams
{
    double v_max = 2.0;
    double a_lat_max = 2.0;
    double a_max = 1.0;
    double a_min = -2.0;
    double jerk_max = 2.0;
    double kp = 0.8;
    double ki = 0.1;
    double kd = 0.0;
    double lpf_vel_error_gain = 0.9;
    double dt = 0.05;
};

class LongitudinalPid
{
public:
    void configure(const LongitudinalParams & params);

    double referenceSpeed(double curvature, double dist_to_end) const;

    double update(double v_ref, double v_meas);

    void reset();

private:
    LongitudinalParams p_{};
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double filtered_error_ = 0.0;
    double prev_v_ref_ = 0.0;
    double prev_a_cmd_ = 0.0;
    double v_cmd_ = 0.0;
    bool initialized_ = false;
};

}
