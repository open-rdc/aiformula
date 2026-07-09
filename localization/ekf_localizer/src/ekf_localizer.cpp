#include "ekf_localizer/ekf_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

#include "utilities/utils.hpp"

namespace ekf_localizer
{
namespace
{

constexpr double HISTORY_RETENTION_SEC = 5.0;

double normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}

double mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance)
{
    return std::sqrt(residual.transpose() * covariance.inverse() * residual);
}

DelayGateResult check_delay_gate(
    const rclcpp::Time& now,
    const rclcpp::Time& stamp,
    const double max_delay_s)
{
    const double raw_delay_time_s = (now - stamp).seconds();
    const double delay_time_s = std::max(raw_delay_time_s, 0.0);
    return DelayGateResult{delay_time_s, delay_time_s <= max_delay_s};
}

EkfLocalizer::EkfLocalizer(const EkfLocalizerConfig& config)
: config_(config),
  state_(Eigen::Vector3d::Zero()),
  covariance_(Eigen::Matrix3d::Identity()),
  stamp_(0, 0, RCL_ROS_TIME),
  initialized_(false)
{
}

bool EkfLocalizer::initialized() const
{
    return initialized_;
}

void EkfLocalizer::initialize(
    const double x,
    const double y,
    const double yaw,
    const rclcpp::Time& stamp)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        throw std::runtime_error("EKF initial state contains non-finite values");
    }

    state_ << x, y, normalize_angle(yaw);
    covariance_.setZero();
    covariance_(0, 0) = config_.initial_position_variance;
    covariance_(1, 1) = config_.initial_position_variance;
    covariance_(2, 2) = config_.initial_yaw_variance;
    stamp_ = stamp;
    initialized_ = true;
    history_.clear();
    record_history();
}

void EkfLocalizer::predict(
    const double velocity,
    const double yaw_rate,
    const rclcpp::Time& stamp)
{
    const double dt = (stamp - stamp_).seconds();
    if (dt < 0.0) {
        throw std::runtime_error("EKF prediction timestamp moved backwards");
    }
    if (dt == 0.0) {
        return;
    }

    const double yaw = state_(2);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    state_(0) += cos_yaw * velocity * dt;
    state_(1) += sin_yaw * velocity * dt;
    state_(2) = normalize_angle(state_(2) + yaw_rate * dt);

    Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
    transition(0, 2) = -sin_yaw * velocity * dt;
    transition(1, 2) = cos_yaw * velocity * dt;

    Eigen::Matrix<double, 3, 2> input_jacobian = Eigen::Matrix<double, 3, 2>::Zero();
    input_jacobian(0, 0) = cos_yaw * dt;
    input_jacobian(1, 0) = sin_yaw * dt;
    input_jacobian(2, 1) = dt;

    Eigen::Matrix2d input_noise = Eigen::Matrix2d::Zero();
    input_noise(0, 0) = config_.process_velocity_variance;
    input_noise(1, 1) = config_.process_yaw_rate_variance;

    Eigen::Matrix3d process_noise = input_jacobian * input_noise * input_jacobian.transpose();
    process_noise(0, 0) += config_.process_position_variance * dt;
    process_noise(1, 1) += config_.process_position_variance * dt;
    process_noise(2, 2) += config_.process_yaw_variance * dt;

    covariance_ = transition * covariance_ * transition.transpose() + process_noise;
    stamp_ = stamp;
    record_history();
}

bool EkfLocalizer::update_position(
    const double x,
    const double y,
    const Eigen::Matrix2d& covariance,
    const rclcpp::Time& stamp)
{
    if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::runtime_error("EKF position measurement contains non-finite values");
    }
    if (!covariance.allFinite() || covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0) {
        throw std::invalid_argument("position measurement covariance must be finite and positive");
    }

    double reference_x = state_(0);
    double reference_y = state_(1);
    double reference_yaw = state_(2);
    pose_at(stamp, reference_x, reference_y, reference_yaw);
    return apply_position_update(
        Eigen::Vector2d(x - reference_x, y - reference_y), covariance);
}

bool EkfLocalizer::apply_position_update(
    const Eigen::Vector2d& residual,
    const Eigen::Matrix2d& covariance)
{
    Eigen::Matrix<double, 2, 3> observation = Eigen::Matrix<double, 2, 3>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;

    const Eigen::Matrix2d innovation_covariance =
        observation * covariance_ * observation.transpose() + covariance;

    if (mahalanobis(residual, innovation_covariance) > config_.position_gate_dist) {
        return false;
    }

    const Eigen::Matrix<double, 3, 2> gain =
        covariance_ * observation.transpose() * innovation_covariance.inverse();

    state_ += gain * residual;
    state_(2) = normalize_angle(state_(2));
    covariance_ = (Eigen::Matrix3d::Identity() - gain * observation) * covariance_;
    record_history();
    return true;
}

bool EkfLocalizer::update_yaw(
    const double yaw,
    const double variance,
    const rclcpp::Time& stamp)
{
    if (!std::isfinite(yaw)) {
        throw std::runtime_error("EKF yaw measurement contains non-finite values");
    }

    double reference_x = state_(0);
    double reference_y = state_(1);
    double reference_yaw = state_(2);
    pose_at(stamp, reference_x, reference_y, reference_yaw);
    return apply_yaw_update(normalize_angle(yaw - reference_yaw), variance);
}

bool EkfLocalizer::apply_yaw_update(
    const double residual,
    const double variance)
{
    Eigen::Matrix<double, 1, 3> observation = Eigen::Matrix<double, 1, 3>::Zero();
    observation(0, 2) = 1.0;

    const double innovation_covariance =
        (observation * covariance_ * observation.transpose())(0, 0) + variance;

    Eigen::VectorXd residual_vec(1);
    residual_vec(0) = residual;
    Eigen::MatrixXd innovation_covariance_mat(1, 1);
    innovation_covariance_mat(0, 0) = innovation_covariance;

    if (mahalanobis(residual_vec, innovation_covariance_mat) > config_.yaw_gate_dist) {
        return false;
    }

    const Eigen::Vector3d gain =
        covariance_ * observation.transpose() / innovation_covariance;

    state_ += gain * residual;
    state_(2) = normalize_angle(state_(2));
    covariance_ = (Eigen::Matrix3d::Identity() - gain * observation) * covariance_;
    record_history();
    return true;
}

bool EkfLocalizer::pose_at(
    const rclcpp::Time& stamp,
    double& x,
    double& y,
    double& yaw) const
{
    if (!initialized_ || history_.empty()) {
        return false;
    }
    if (stamp <= history_.front().stamp) {
        x = history_.front().x;
        y = history_.front().y;
        yaw = history_.front().yaw;
        return true;
    }
    if (stamp >= history_.back().stamp) {
        x = history_.back().x;
        y = history_.back().y;
        yaw = history_.back().yaw;
        return true;
    }

    const auto upper = std::lower_bound(
        history_.begin(), history_.end(), stamp,
        [](const HistoryEntry& entry, const rclcpp::Time& query) {
            return entry.stamp < query;
        });
    const auto lower = upper - 1;

    const double span = (upper->stamp - lower->stamp).seconds();
    const double ratio = span <= 0.0 ? 1.0 : (stamp - lower->stamp).seconds() / span;
    x = lower->x + ratio * (upper->x - lower->x);
    y = lower->y + ratio * (upper->y - lower->y);
    yaw = normalize_angle(lower->yaw + ratio * normalize_angle(upper->yaw - lower->yaw));
    return true;
}

geometry_msgs::msg::PoseWithCovarianceStamped EkfLocalizer::make_pose(
    const std::string& frame_id) const
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp_;
    pose.header.frame_id = frame_id;
    pose.pose.pose.position.x = state_(0);
    pose.pose.pose.position.y = state_(1);
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = utils::yaw_to_quaternion(state_(2));
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = covariance_(0, 0);
    pose.pose.covariance[1] = covariance_(0, 1);
    pose.pose.covariance[6] = covariance_(1, 0);
    pose.pose.covariance[7] = covariance_(1, 1);
    pose.pose.covariance[35] = covariance_(2, 2);
    return pose;
}

void EkfLocalizer::record_history()
{
    if (!history_.empty() && history_.back().stamp == stamp_) {
        history_.back() = HistoryEntry{stamp_, state_(0), state_(1), state_(2)};
    } else {
        history_.push_back(HistoryEntry{stamp_, state_(0), state_(1), state_(2)});
    }
    while (history_.size() > 1U &&
           (stamp_ - history_.front().stamp).seconds() > HISTORY_RETENTION_SEC)
    {
        history_.pop_front();
    }
}

}
