#include "frenet_planner/reference_curve.hpp"

#include <algorithm>
#include <cmath>

namespace frenet_planner {

ReferenceCurve::ReferenceCurve(const std::vector<double>& x, const std::vector<double>& y)
    : x_(x), y_(y) {
    const size_t n = x.size();
    s_.resize(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        double dx = x_[i] - x_[i - 1];
        double dy = y_[i] - y_[i - 1];
        s_[i] = s_[i - 1] + std::hypot(dx, dy);
    }

    yaw_.resize(n, 0.0);
    kappa_.resize(n, 0.0);
    dkappa_.resize(n, 0.0);

    std::vector<double> dx_ds(n, 0.0);
    std::vector<double> dy_ds(n, 0.0);
    std::vector<double> ddx_ds(n, 0.0);
    std::vector<double> ddy_ds(n, 0.0);

    for (size_t i = 1; i + 1 < n; ++i) {
        const double ds_prev = std::max(1e-6, s_[i] - s_[i - 1]);
        const double ds_next = std::max(1e-6, s_[i + 1] - s_[i]);
        const double ds_sum = ds_prev + ds_next;

        dx_ds[i] = (x_[i + 1] - x_[i - 1]) / ds_sum;
        dy_ds[i] = (y_[i + 1] - y_[i - 1]) / ds_sum;

        const double dx_prev = (x_[i] - x_[i - 1]) / ds_prev;
        const double dx_next = (x_[i + 1] - x_[i]) / ds_next;
        const double dy_prev = (y_[i] - y_[i - 1]) / ds_prev;
        const double dy_next = (y_[i + 1] - y_[i]) / ds_next;

        ddx_ds[i] = 2.0 * (dx_next - dx_prev) / ds_sum;
        ddy_ds[i] = 2.0 * (dy_next - dy_prev) / ds_sum;
    }

    dx_ds[0] = (x_[1] - x_[0]) / std::max(1e-6, s_[1] - s_[0]);
    dy_ds[0] = (y_[1] - y_[0]) / std::max(1e-6, s_[1] - s_[0]);
    dx_ds[n - 1] = (x_[n - 1] - x_[n - 2]) / std::max(1e-6, s_[n - 1] - s_[n - 2]);
    dy_ds[n - 1] = (y_[n - 1] - y_[n - 2]) / std::max(1e-6, s_[n - 1] - s_[n - 2]);

    ddx_ds[0] = ddx_ds[1];
    ddy_ds[0] = ddy_ds[1];
    ddx_ds[n - 1] = ddx_ds[n - 2];
    ddy_ds[n - 1] = ddy_ds[n - 2];

    for (size_t i = 0; i < n; ++i) {
        yaw_[i] = std::atan2(dy_ds[i], dx_ds[i]);
        const double v2 = dx_ds[i] * dx_ds[i] + dy_ds[i] * dy_ds[i];
        const double v = std::sqrt(v2);
        if (v < 1e-6) {
            kappa_[i] = 0.0;
        } else {
            kappa_[i] = (dx_ds[i] * ddy_ds[i] - dy_ds[i] * ddx_ds[i]) / (v2 * v);
        }
    }

    for (size_t i = 1; i + 1 < n; ++i) {
        const double ds_prev = std::max(1e-6, s_[i] - s_[i - 1]);
        const double ds_next = std::max(1e-6, s_[i + 1] - s_[i]);
        dkappa_[i] = (kappa_[i + 1] - kappa_[i - 1]) / (ds_prev + ds_next);
    }
    dkappa_[0] = dkappa_[1];
    dkappa_[n - 1] = dkappa_[n - 2];

    valid_ = true;
}

bool ReferenceCurve::valid() const {
    return valid_;
}

double ReferenceCurve::max_s() const {
    if (s_.empty()) {
        return 0.0;
    }
    return s_.back();
}

void ReferenceCurve::sample(double s,
                            double& x,
                            double& y,
                            double& yaw,
                            double& kappa,
                            double& dkappa) const {
    double clamped_s = std::max(0.0, std::min(s, s_.back()));
    auto it = std::upper_bound(s_.begin(), s_.end(), clamped_s);
    size_t i = 0;
    if (it != s_.begin()) {
        i = static_cast<size_t>(it - s_.begin()) - 1;
    }
    if (i + 1 >= s_.size()) {
        i = s_.size() - 2;
    }

    const double s0 = s_[i];
    const double s1 = s_[i + 1];
    const double denom = std::max(1e-6, s1 - s0);
    const double t = (clamped_s - s0) / denom;

    auto lerp = [t](double a, double b) {
        return a + (b - a) * t;
    };

    x = lerp(x_[i], x_[i + 1]);
    y = lerp(y_[i], y_[i + 1]);
    yaw = lerp(yaw_[i], yaw_[i + 1]);
    kappa = lerp(kappa_[i], kappa_[i + 1]);
    dkappa = lerp(dkappa_[i], dkappa_[i + 1]);
}

}  // namespace frenet_planner
