#pragma once

namespace ekf_localizer {

struct VelocityGateResult {
    bool   passed;
    double velocity;
    double yaw_rate;
};

class VelocityGate {
   public:
    VelocityGate (double process_velocity_variance, double process_yaw_rate_variance, double gate_dist, double min_velocity_variance, double min_yaw_rate_variance, double max_reject_duration_s);

    VelocityGateResult update (double velocity, double yaw_rate, double velocity_variance, double yaw_rate_variance, double dt);

   private:
    double process_velocity_variance_;
    double process_yaw_rate_variance_;
    double gate_dist_;
    double min_velocity_variance_;
    double min_yaw_rate_variance_;
    double max_reject_duration_s_;
    bool   initialized_;
    double velocity_;
    double velocity_variance_;
    double yaw_rate_;
    double yaw_rate_variance_;
    double rejected_elapsed_s_ = 0.0;
};

}  // namespace ekf_localizer
