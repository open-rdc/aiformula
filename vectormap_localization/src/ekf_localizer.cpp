#include "vectormap_localization/ekf_localizer.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

namespace vectormap_localization
{

EkfLocalizer::EkfLocalizer(const EkfLocalizerConfig& config)
: config_(config),
  state_(Eigen::Matrix<double, 5, 1>::Zero()),
  covariance_(Eigen::Matrix<double, 5, 5>::Identity()),
  stamp_(0, 0, RCL_ROS_TIME),
  initialized_(false)
{
    validate_variance(config_.initial_position_variance, "ekf.initial_position_variance");
    validate_variance(config_.initial_yaw_variance, "ekf.initial_yaw_variance");
    validate_variance(config_.initial_velocity_variance, "ekf.initial_velocity_variance");
    validate_variance(config_.initial_yaw_rate_variance, "ekf.initial_yaw_rate_variance");
    validate_variance(config_.process_position_variance, "ekf.process_position_variance");
    validate_variance(config_.process_yaw_variance, "ekf.process_yaw_variance");
    validate_variance(config_.process_velocity_variance, "ekf.process_velocity_variance");
    validate_variance(config_.process_yaw_rate_variance, "ekf.process_yaw_rate_variance");
    validate_variance(config_.gnss_position_variance, "ekf.gnss_position_variance");
    validate_variance(config_.imu_yaw_variance, "ekf.imu_yaw_variance");
    validate_variance(config_.icp_position_variance, "ekf.icp_position_variance");
}

bool EkfLocalizer::initialized() const
{
    return initialized_;
}

void EkfLocalizer::initialize(
    const double x,
    const double y,
    const double yaw,
    const double velocity,
    const double yaw_rate,
    const rclcpp::Time& stamp)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(velocity) || !std::isfinite(yaw_rate))
    {
        throw std::runtime_error("EKF initial state contains non-finite values");
    }

    state_ << x, y, normalize_angle(yaw), velocity, yaw_rate;
    covariance_.setZero();
    covariance_(0, 0) = config_.initial_position_variance;
    covariance_(1, 1) = config_.initial_position_variance;
    covariance_(2, 2) = config_.initial_yaw_variance;
    covariance_(3, 3) = config_.initial_velocity_variance;
    covariance_(4, 4) = config_.initial_yaw_rate_variance;
    stamp_ = stamp;
    initialized_ = true;
}

void EkfLocalizer::predict(
    const double velocity,
    const double yaw_rate,
    const rclcpp::Time& stamp)
{
    if (!initialized_) {
        throw std::runtime_error("EKF predict called before initialization");
    }
    if (!std::isfinite(velocity) || !std::isfinite(yaw_rate)) {
        throw std::runtime_error("EKF prediction input contains non-finite values");
    }

    const double dt = (stamp - stamp_).seconds();
    if (dt < 0.0) {
        throw std::runtime_error("EKF prediction timestamp moved backwards");
    }
    if (dt == 0.0) {
        state_(3) = velocity;
        state_(4) = yaw_rate;
        return;
    }

    const double yaw = state_(2);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    state_(3) = velocity;
    state_(4) = yaw_rate;
    state_(0) += cos_yaw * state_(3) * dt;
    state_(1) += sin_yaw * state_(3) * dt;
    state_(2) = normalize_angle(state_(2) + state_(4) * dt);

    Eigen::Matrix<double, 5, 5> transition = Eigen::Matrix<double, 5, 5>::Identity();
    transition(0, 2) = -sin_yaw * state_(3) * dt;
    transition(0, 3) = cos_yaw * dt;
    transition(1, 2) = cos_yaw * state_(3) * dt;
    transition(1, 3) = sin_yaw * dt;
    transition(2, 4) = dt;

    Eigen::Matrix<double, 5, 5> process_noise = Eigen::Matrix<double, 5, 5>::Zero();
    process_noise(0, 0) = config_.process_position_variance * dt;
    process_noise(1, 1) = config_.process_position_variance * dt;
    process_noise(2, 2) = config_.process_yaw_variance * dt;
    process_noise(3, 3) = config_.process_velocity_variance * dt;
    process_noise(4, 4) = config_.process_yaw_rate_variance * dt;

    covariance_ = transition * covariance_ * transition.transpose() + process_noise;
    stamp_ = stamp;
}

void EkfLocalizer::update_position(
    const double x,
    const double y,
    const double variance)
{
    if (!initialized_) {
        throw std::runtime_error("EKF position update called before initialization");
    }
    if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::runtime_error("EKF position measurement contains non-finite values");
    }
    validate_variance(variance, "position measurement variance");

    Eigen::Matrix<double, 2, 5> observation = Eigen::Matrix<double, 2, 5>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;

    Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Identity() * variance;
    const Eigen::Vector2d residual(x - state_(0), y - state_(1));
    const Eigen::Matrix2d innovation_covariance =
        observation * covariance_ * observation.transpose() + measurement_noise;
    const Eigen::Matrix<double, 5, 2> gain =
        covariance_ * observation.transpose() * innovation_covariance.inverse();

    state_ += gain * residual;
    state_(2) = normalize_angle(state_(2));
    const Eigen::Matrix<double, 5, 5> identity = Eigen::Matrix<double, 5, 5>::Identity();
    covariance_ = (identity - gain * observation) * covariance_;
}

void EkfLocalizer::update_yaw(
    const double yaw,
    const double variance)
{
    if (!initialized_) {
        throw std::runtime_error("EKF yaw update called before initialization");
    }
    if (!std::isfinite(yaw)) {
        throw std::runtime_error("EKF yaw measurement contains non-finite values");
    }
    validate_variance(variance, "yaw measurement variance");

    Eigen::Matrix<double, 1, 5> observation = Eigen::Matrix<double, 1, 5>::Zero();
    observation(0, 2) = 1.0;

    const double residual = normalize_angle(yaw - state_(2));
    const double innovation_covariance =
        (observation * covariance_ * observation.transpose())(0, 0) + variance;
    const Eigen::Matrix<double, 5, 1> gain =
        covariance_ * observation.transpose() / innovation_covariance;

    state_ += gain * residual;
    state_(2) = normalize_angle(state_(2));
    const Eigen::Matrix<double, 5, 5> identity = Eigen::Matrix<double, 5, 5>::Identity();
    covariance_ = (identity - gain * observation) * covariance_;
}

geometry_msgs::msg::PoseWithCovarianceStamped EkfLocalizer::make_pose(
    const rclcpp::Time& stamp,
    const std::string& frame_id) const
{
    if (!initialized_) {
        throw std::runtime_error("EKF pose requested before initialization");
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;
    pose.pose.pose.position.x = state_(0);
    pose.pose.pose.position.y = state_(1);
    pose.pose.pose.position.z = 0.0;
    set_yaw(pose.pose.pose.orientation, state_(2));
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = covariance_(0, 0);
    pose.pose.covariance[1] = covariance_(0, 1);
    pose.pose.covariance[6] = covariance_(1, 0);
    pose.pose.covariance[7] = covariance_(1, 1);
    pose.pose.covariance[35] = covariance_(2, 2);
    return pose;
}

double EkfLocalizer::normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void EkfLocalizer::set_yaw(
    geometry_msgs::msg::Quaternion& quaternion,
    const double yaw)
{
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
}

void EkfLocalizer::validate_variance(
    const double variance,
    const char* const name)
{
    if (!std::isfinite(variance) || variance <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and greater than 0");
    }
}

}  // namespace vectormap_localization
