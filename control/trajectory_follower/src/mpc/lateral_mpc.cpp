#include "trajectory_follower/mpc/lateral_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "trajectory_follower/mpc/speed_limit.hpp"

namespace trajectory_follower
{

namespace
{
double normalize_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}

void LateralMpc::configure(const LateralMpcParams & params)
{
    params_ = params;
    reset();
}

double LateralMpc::computeSteering(
    const std::vector<std::array<double, 2>> & path_xy, double v)
{
    const int n = static_cast<int>(path_xy.size());
    if (n < 3) {
        return std::clamp(prev_steer_, -params_.steer_limit, params_.steer_limit);
    }

    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i) {
        arc[i] = arc[i - 1] + std::hypot(path_xy[i][0] - path_xy[i - 1][0],
                                         path_xy[i][1] - path_xy[i - 1][1]);
    }
    std::vector<double> curv(n, 0.0);
    for (int i = 1; i < n - 1; ++i) {
        curv[i] = menger_curvature(path_xy[i - 1], path_xy[i], path_xy[i + 1]);
    }
    curv[0] = curv[1];
    curv[n - 1] = curv[n - 2];

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
    const double theta = std::atan2(path_xy[hi][1] - path_xy[lo][1],
                                    path_xy[hi][0] - path_xy[lo][0]);

    const double x0 = path_xy[nearest][0];
    const double y0 = path_xy[nearest][1];
    const double e_y = std::sin(theta) * x0 - std::cos(theta) * y0;
    const double e_yaw = normalize_angle(0.0 - theta);

    const int N = std::max(1, params_.horizon);
    const double dt = params_.prediction_dt;
    const double V = std::max(v, params_.min_predict_speed);
    const double L = params_.wheelbase;
    const double tau = std::max(params_.steer_tau, 1.0e-3);
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
        return curv[j];
    };

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

    double steer_state = prev_steer_;
    if (measured_steer_) {
        const double normalized =
            std::atan2(std::sin(*measured_steer_), std::cos(*measured_steer_));
        steer_state = std::clamp(normalized, -params_.steer_limit, params_.steer_limit);
        measured_steer_.reset();
    }

    const Eigen::VectorXd x0vec = (Eigen::Vector3d() << e_y, e_yaw, steer_state).finished();

    const Eigen::MatrixXd SuTQ = Su.transpose() * Qbar;
    Eigen::MatrixXd M = SuTQ * Su + R * Eigen::MatrixXd::Identity(N, N) + Rd * (D.transpose() * D);
    const Eigen::VectorXd rhs = -(SuTQ * (Sx * x0vec + Sw)) + Rd * (D.transpose() * pvec);

    const Eigen::VectorXd U = M.ldlt().solve(rhs);

    double steer = U(0);
    if (!std::isfinite(steer)) {
        steer = prev_steer_;
    }
    steer = std::clamp(steer, -params_.steer_limit, params_.steer_limit);
    prev_steer_ = steer;
    return steer;
}

void LateralMpc::reset()
{
    prev_steer_ = 0.0;
    measured_steer_.reset();
}

void LateralMpc::setMeasuredSteer(double steer)
{
    measured_steer_ = steer;
}

}
