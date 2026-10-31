#include "trajectory_follower/pid_mpc/lateral_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace trajectory_follower
{

TrackingError computeTrackingError(
    const std::vector<std::array<double, 2>> & path_xy,
    const std::vector<double> & curvatures)
{
    const int n = static_cast<int>(path_xy.size());
    int nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        const double d = path_xy[i][0] * path_xy[i][0] + path_xy[i][1] * path_xy[i][1];
        if (d < best) {
            best = d;
            nearest = i;
        }
    }

    const int hi = (nearest < n - 1) ? nearest + 1 : nearest;
    const int lo = (nearest < n - 1) ? nearest : nearest - 1;
    const double chord_theta = std::atan2(path_xy[hi][1] - path_xy[lo][1],
                                          path_xy[hi][0] - path_xy[lo][0]);
    const double chord_len = std::hypot(path_xy[hi][0] - path_xy[lo][0],
                                        path_xy[hi][1] - path_xy[lo][1]);
    // 弦の向きは弦中点の接線方向なので、曲率で最近傍点上の向きに補正する
    const double kappa_n = curvatures[nearest];
    const double theta = std::remainder(
        chord_theta + ((nearest < n - 1) ? -0.5 : 0.5) * kappa_n * chord_len, 2.0 * M_PI);

    const double x0 = path_xy[nearest][0];
    const double y0 = path_xy[nearest][1];
    TrackingError err;
    err.nearest = nearest;
    err.e_y = std::sin(theta) * x0 - std::cos(theta) * y0;
    err.e_yaw = std::remainder(0.0 - theta, 2.0 * M_PI);

    // 経路が自車より前から始まる場合は、定曲率の円弧で垂線の足まで外挿する
    const double a_fwd = x0 * std::cos(theta) + y0 * std::sin(theta);
    if (nearest == 0 && a_fwd > 0.0) {
        err.e_yaw = std::remainder(-theta + kappa_n * a_fwd, 2.0 * M_PI);
        err.e_y -= 0.5 * kappa_n * a_fwd * a_fwd;
    }
    return err;
}

void LateralMpc::configure(const LateralMpcParams & params)
{
    params_ = params;
    reset();
}

double LateralMpc::computeSteering(
    const std::vector<std::array<double, 2>> & path_xy,
    const std::vector<double> & curvatures,
    double v)
{
    const int n = static_cast<int>(path_xy.size());

    // 0. 直進中にヨーレートから舵角オフセットを推定する
    if (has_yaw_rate_ && v > params_.offset_update_vel_threshold &&
        std::abs(steer_state_) < params_.offset_update_steer_threshold) {
        const double sample = std::atan(params_.wheelbase * yaw_rate_ / v) - steer_state_;
        steering_offset_ += params_.control_dt / params_.offset_time_constant * (sample - steering_offset_);
        steering_offset_ = std::clamp(steering_offset_, -params_.offset_limit, params_.offset_limit);
    }
    has_yaw_rate_ = false;

    // 1. 経路上の追従誤差を求める
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i) {
        arc[i] = arc[i - 1] + std::hypot(path_xy[i][0] - path_xy[i - 1][0],
                                         path_xy[i][1] - path_xy[i - 1][1]);
    }
    const TrackingError err = computeTrackingError(path_xy, curvatures);
    const int nearest = err.nearest;
    const double e_y = err.e_y;
    const double e_yaw = err.e_yaw;

    // 2. 離散化モデルを作る
    const int N = params_.horizon;
    const double dt = params_.prediction_dt;
    const double V = std::max(v, params_.min_predict_speed);
    const double L = params_.wheelbase;
    const double tau = params_.steer_tau;
    const double ds = V * dt;

    Eigen::Matrix3d Ac;
    Ac << 0.0, V, 0.0,
          0.0, 0.0, V / L,
          0.0, 0.0, -1.0 / tau;
    const Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity() + Ac * dt;
    Eigen::Vector3d Bc(0.0, 0.0, 1.0 / tau);
    const Eigen::Vector3d Bd = Bc * dt;

    auto curvature_at = [&](double s) -> double {
        const double s_abs = arc[nearest] + s;
        int j = nearest;
        while (j < n - 1 && arc[j] < s_abs) ++j;
        return curvatures[j];
    };

    // 3. 予測行列を作る
    std::vector<Eigen::Matrix3d> Apow(N + 1, Eigen::Matrix3d::Identity());
    for (int k = 1; k <= N; ++k) {
        Apow[k] = Apow[k - 1] * Ad;
    }

    Eigen::MatrixXd Sx(3 * N, 3);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(3 * N, N);
    Eigen::VectorXd Sw = Eigen::VectorXd::Zero(3 * N);

    std::vector<Eigen::Vector3d> wd(N, Eigen::Vector3d::Zero());
    for (int i = 0; i < N; ++i) {
        const double kappa = curvature_at(static_cast<double>(i) * ds);
        wd[i] = Eigen::Vector3d(0.0, -V * kappa, 0.0) * dt;
    }

    for (int k = 1; k <= N; ++k) {
        const int r = (k - 1) * 3;
        Sx.block<3, 3>(r, 0) = Apow[k];
        for (int j = 0; j < k; ++j) {
            Su.block<3, 1>(r, j) = Apow[k - 1 - j] * Bd;
        }
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        for (int i = 0; i < k; ++i) {
            acc += Apow[k - 1 - i] * wd[i];
        }
        Sw.segment<3>(r) = acc;
    }

    // 4. コスト行列を作る
    Eigen::VectorXd qdiag(3 * N);
    for (int k = 1; k <= N; ++k) {
        const int r = (k - 1) * 3;
        const bool terminal = (k == N);
        qdiag(r) = terminal ? params_.weight_terminal_lat_error : params_.weight_lat_error;
        qdiag(r + 1) = terminal ? params_.weight_terminal_heading_error : params_.weight_heading_error;
        qdiag(r + 2) = 0.0;
    }
    const Eigen::MatrixXd Qbar = qdiag.asDiagonal();

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(N, N);
    Eigen::VectorXd pvec = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < N; ++k) {
        D(k, k) = 1.0;
        if (k > 0) D(k, k - 1) = -1.0;
    }
    pvec(0) = prev_steer_;

    const double R = params_.weight_steering_input;
    const double Rd = params_.weight_steer_rate;

    const Eigen::VectorXd x0vec = (Eigen::Vector3d() << e_y, e_yaw, steer_state_ + steering_offset_).finished();

    // 5. QP を解く
    const Eigen::MatrixXd SuTQ = Su.transpose() * Qbar;
    Eigen::MatrixXd M = SuTQ * Su + R * Eigen::MatrixXd::Identity(N, N) + Rd * (D.transpose() * D);
    const Eigen::VectorXd rhs = -(SuTQ * (Sx * x0vec + Sw)) + Rd * (D.transpose() * pvec);

    const Eigen::VectorXd U = M.ldlt().solve(rhs);

    const double steer = std::clamp(U(0), -params_.steer_limit, params_.steer_limit);
    prev_steer_ = steer;
    steer_state_ = std::clamp(steer - steering_offset_, -params_.steer_limit, params_.steer_limit);
    return steer_state_;
}

void LateralMpc::reset()
{
    prev_steer_ = 0.0;
    steer_state_ = 0.0;
}

void LateralMpc::setMeasuredSteer(double steer)
{
    steer_state_ = std::clamp(std::remainder(steer, 2.0 * M_PI), -params_.steer_limit, params_.steer_limit);
}

void LateralMpc::setYawRate(double yaw_rate)
{
    yaw_rate_ = yaw_rate;
    has_yaw_rate_ = true;
}

}
