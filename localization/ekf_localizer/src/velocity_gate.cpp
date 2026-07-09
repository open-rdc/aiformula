#include "ekf_localizer/velocity_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>

#include "ekf_localizer/mahalanobis.hpp"

namespace ekf_localizer
{

VelocityGate::VelocityGate(
    const double process_velocity_variance,
    const double process_yaw_rate_variance,
    const double gate_dist)
: process_velocity_variance_(process_velocity_variance),
  process_yaw_rate_variance_(process_yaw_rate_variance),
  gate_dist_(gate_dist),
  initialized_(false),
  velocity_(0.0),
  velocity_variance_(0.0),
  yaw_rate_(0.0),
  yaw_rate_variance_(0.0)
{
}

VelocityGateResult VelocityGate::update(
    const double velocity,
    const double yaw_rate,
    const double velocity_variance,
    const double yaw_rate_variance,
    const double dt)
{
    if (!std::isfinite(velocity_variance) || !std::isfinite(yaw_rate_variance) ||
        velocity_variance <= 0.0 || yaw_rate_variance <= 0.0)
    {
        throw std::invalid_argument("velocity measurement covariance must be finite and positive");
    }

    if (!initialized_) {
        velocity_ = velocity;
        velocity_variance_ = velocity_variance;
        yaw_rate_ = yaw_rate;
        yaw_rate_variance_ = yaw_rate_variance;
        initialized_ = true;
        return VelocityGateResult{true, velocity_, yaw_rate_};
    }

    const double predicted_velocity_variance =
        velocity_variance_ + process_velocity_variance_ * std::max(dt, 0.0);
    const double predicted_yaw_rate_variance =
        yaw_rate_variance_ + process_yaw_rate_variance_ * std::max(dt, 0.0);

    Eigen::VectorXd residual(2);
    residual(0) = velocity - velocity_;
    residual(1) = yaw_rate - yaw_rate_;

    Eigen::MatrixXd innovation_covariance = Eigen::MatrixXd::Zero(2, 2);
    innovation_covariance(0, 0) = predicted_velocity_variance + velocity_variance;
    innovation_covariance(1, 1) = predicted_yaw_rate_variance + yaw_rate_variance;

    if (mahalanobis(residual, innovation_covariance) > gate_dist_) {
        return VelocityGateResult{false, velocity_, yaw_rate_};
    }

    const double kv =
        predicted_velocity_variance / (predicted_velocity_variance + velocity_variance);
    velocity_ += kv * residual(0);
    velocity_variance_ = (1.0 - kv) * predicted_velocity_variance;

    const double kw =
        predicted_yaw_rate_variance / (predicted_yaw_rate_variance + yaw_rate_variance);
    yaw_rate_ += kw * residual(1);
    yaw_rate_variance_ = (1.0 - kw) * predicted_yaw_rate_variance;

    return VelocityGateResult{true, velocity_, yaw_rate_};
}

}
