#include "yolopnav/lane_kalman_filter.hpp"
#include <cmath>

namespace yolopnav {

LaneKalmanFilter::LaneKalmanFilter()
    : state_(Eigen::Vector4d::Zero()),
      covariance_(Eigen::Matrix4d::Identity()),
      observation_noise_(0.1),
      initialized_(false) {

    process_noise_ = Eigen::Matrix4d::Identity() * 0.001;
}

void LaneKalmanFilter::predict() {
    if (!initialized_) return;

    covariance_ += process_noise_;
}

void LaneKalmanFilter::update(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() < 4) return;

    if (!initialized_) {
        initialize(points);
        return;
    }

    int n = points.size();

    Eigen::MatrixXd H(n, 4);
    Eigen::VectorXd z(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].x();
        double y = points[i].y();

        H(i, 0) = 1.0;
        H(i, 1) = x;
        H(i, 2) = x * x;
        H(i, 3) = x * x * x;

        z(i) = y;
    }

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n, n) * (observation_noise_ * observation_noise_);
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    Eigen::VectorXd innovation = z - H * state_;
    state_ += K * innovation;

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    covariance_ = (I - K * H) * covariance_;
}

double LaneKalmanFilter::evaluate(double x) const {
    if (!initialized_) return 0.0;

    // y = a0 + a1*x + a2*x^2 + a3*x^3
    return state_(0) + state_(1) * x + state_(2) * x * x + state_(3) * x * x * x;
}

void LaneKalmanFilter::initialize(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() < 4) return;

    int n = points.size();
    Eigen::MatrixXd A(n, 4);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].x();
        double y = points[i].y();

        A(i, 0) = 1.0;
        A(i, 1) = x;
        A(i, 2) = x * x;
        A(i, 3) = x * x * x;

        b(i) = y;
    }

    state_ = A.colPivHouseholderQr().solve(b);
    covariance_ = Eigen::Matrix4d::Identity() * 1.0;
    initialized_ = true;
}


// LaneKalmanManager implementation
LaneKalmanManager::LaneKalmanManager() {}

void LaneKalmanManager::updateLeft(const std::vector<Eigen::Vector3d>& points) {
    left_filter_.update(points);
}

void LaneKalmanManager::updateRight(const std::vector<Eigen::Vector3d>& points) {
    right_filter_.update(points);
}

void LaneKalmanManager::updateCenter(const std::vector<Eigen::Vector3d>& points) {
    center_filter_.update(points);
}

void LaneKalmanManager::predict() {
    left_filter_.predict();
    right_filter_.predict();
    center_filter_.predict();
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getLeftLanePoints(double interval, double min_x, double max_x) const {
    return generatePoints(left_filter_, interval, min_x, max_x);
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getRightLanePoints(double interval, double min_x, double max_x) const {
    return generatePoints(right_filter_, interval, min_x, max_x);
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getCenterLanePoints(double interval, double min_x, double max_x) const {
    return generatePoints(center_filter_, interval, min_x, max_x);
}

std::vector<Eigen::Vector3d> LaneKalmanManager::generatePoints(const LaneKalmanFilter& filter,
                                                                 double interval, double min_x, double max_x) const {
    std::vector<Eigen::Vector3d> points;
    if (!filter.isInitialized()) return points;

    for (double x = min_x; x <= max_x; x += interval) {
        double y = filter.evaluate(x);
        points.emplace_back(x, y, 0.0);
    }

    return points;
}

}  // namespace yolopnav
