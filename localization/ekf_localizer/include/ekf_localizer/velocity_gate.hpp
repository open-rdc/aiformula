#pragma once

namespace ekf_localizer
{

struct VelocityGateResult
{
    bool passed;
    double velocity;
    double yaw_rate;
};

class VelocityGate
{
public:
    VelocityGate(
        double process_velocity_variance,
        double process_yaw_rate_variance,
        double gate_dist);

    VelocityGateResult update(
        double velocity,
        double yaw_rate,
        double velocity_variance,
        double yaw_rate_variance,
        double dt);

private:
    double process_velocity_variance_;
    double process_yaw_rate_variance_;
    double gate_dist_;
    bool initialized_;
    double velocity_;
    double velocity_variance_;
    double yaw_rate_;
    double yaw_rate_variance_;
};

}
