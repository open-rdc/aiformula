#include "ekf_localizer/velocity_gate.hpp"

#include "ekf_localizer/ekf_localizer.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace ekf_localizer {

VelocityGate::VelocityGate (const double process_velocity_variance, const double process_yaw_rate_variance, const double gate_dist, const double min_velocity_variance, const double min_yaw_rate_variance, const double max_reject_duration_s)
    : process_velocity_variance_ (process_velocity_variance),
      process_yaw_rate_variance_ (process_yaw_rate_variance),
      gate_dist_ (gate_dist),
      min_velocity_variance_ (min_velocity_variance),
      min_yaw_rate_variance_ (min_yaw_rate_variance),
      max_reject_duration_s_ (max_reject_duration_s),
      initialized_ (false),
      velocity_ (0.0),
      velocity_variance_ (0.0),
      yaw_rate_ (0.0),
      yaw_rate_variance_ (0.0) {}

VelocityGateResult VelocityGate::update (const double velocity, const double yaw_rate, const double velocity_variance, const double yaw_rate_variance, const double dt) {
    if (!initialized_) {
        velocity_          = velocity;
        velocity_variance_ = std::max (velocity_variance, min_velocity_variance_);
        yaw_rate_          = yaw_rate;
        yaw_rate_variance_ = std::max (yaw_rate_variance, min_yaw_rate_variance_);
        initialized_       = true;
        return VelocityGateResult{true, velocity_, yaw_rate_};
    }

    const double predicted_velocity_variance = velocity_variance_ + process_velocity_variance_ * dt;
    const double predicted_yaw_rate_variance = yaw_rate_variance_ + process_yaw_rate_variance_ * dt;

    Eigen::VectorXd residual (2);
    residual (0) = velocity - velocity_;
    residual (1) = yaw_rate - yaw_rate_;

    Eigen::MatrixXd innovation_covariance = Eigen::MatrixXd::Zero (2, 2);
    innovation_covariance (0, 0)          = predicted_velocity_variance + velocity_variance;
    innovation_covariance (1, 1)          = predicted_yaw_rate_variance + yaw_rate_variance;

    // 連続棄却の経過時間がタイムアウトを超えたら、EKFが静止したまま実位置と
    // 乖離し続ける危険を避けるためMahalanobisゲートをスキップして強制受理する。
    const bool force_accept = rejected_elapsed_s_ >= max_reject_duration_s_;

    if (!force_accept && mahalanobis (residual, innovation_covariance) > gate_dist_) {
        velocity_variance_ = std::max (predicted_velocity_variance, min_velocity_variance_);
        yaw_rate_variance_ = std::max (predicted_yaw_rate_variance, min_yaw_rate_variance_);
        rejected_elapsed_s_ += dt;
        return VelocityGateResult{false, velocity_, yaw_rate_};
    }

    const double kv = predicted_velocity_variance / (predicted_velocity_variance + velocity_variance);
    velocity_ += kv * residual (0);
    // 停止継続などで一致する観測が連続すると分散が際限なく収縮し、
    // 直後の急発進のような正しい観測までゲートロックで弾かれ続けるため下限を設ける。
    velocity_variance_ = std::max ((1.0 - kv) * predicted_velocity_variance, min_velocity_variance_);

    const double kw = predicted_yaw_rate_variance / (predicted_yaw_rate_variance + yaw_rate_variance);
    yaw_rate_ += kw * residual (1);
    yaw_rate_variance_ = std::max ((1.0 - kw) * predicted_yaw_rate_variance, min_yaw_rate_variance_);

    rejected_elapsed_s_ = 0.0;
    return VelocityGateResult{true, velocity_, yaw_rate_};
}

}  // namespace ekf_localizer
