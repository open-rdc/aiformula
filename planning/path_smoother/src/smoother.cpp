#include "path_smoother/smoother.hpp"

#include <cmath>
#include <random>

namespace path_smoother {

namespace {

constexpr double MAX_TANGENT_RAD = 30.0 * M_PI / 180.0;

double det3(const double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

}  // namespace

std::vector<Point2D> to_odom(const std::vector<Point2D>& points_base, const Pose2D& odom_T_base) {
    const double         c = std::cos(odom_T_base.yaw);
    const double         s = std::sin(odom_T_base.yaw);
    std::vector<Point2D> out;
    out.reserve(points_base.size());
    for (const auto& p : points_base) {
        out.push_back(Point2D{odom_T_base.x + c * p.x - s * p.y, odom_T_base.y + s * p.x + c * p.y});
    }
    return out;
}

std::vector<Point2D> to_base(const std::vector<Point2D>& points_odom, const Pose2D& odom_T_base) {
    const double         c = std::cos(odom_T_base.yaw);
    const double         s = std::sin(odom_T_base.yaw);
    std::vector<Point2D> out;
    out.reserve(points_odom.size());
    for (const auto& p : points_odom) {
        const double dx = p.x - odom_T_base.x;
        const double dy = p.y - odom_T_base.y;
        out.push_back(Point2D{c * dx + s * dy, -s * dx + c * dy});
    }
    return out;
}

std::vector<double> arc_weights(const std::vector<Point2D>& points) {
    const std::size_t   n = points.size();
    std::vector<double> weights(n, 0.0);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const double half = 0.5 * std::hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
        weights[i] += half;
        weights[i + 1] += half;
    }
    return weights;
}

Quadratic fit_quadratic(const std::vector<Point2D>& points, const std::vector<double>& weights) {
    // 正規方程式 (Σ w x^(i+j)) c = Σ w y x^i をクラメルで解く
    double s[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double t[3] = {0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < points.size(); ++i) {
        double xk = 1.0;
        for (int k = 0; k < 5; ++k) {
            s[k] += weights[i] * xk;
            if (k < 3) {
                t[k] += weights[i] * points[i].y * xk;
            }
            xk *= points[i].x;
        }
    }
    const double a[3][3] = {{s[0], s[1], s[2]}, {s[1], s[2], s[3]}, {s[2], s[3], s[4]}};
    const double det     = det3(a);
    Quadratic    c{};
    for (int j = 0; j < 3; ++j) {
        double m[3][3];
        for (int r = 0; r < 3; ++r) {
            for (int col = 0; col < 3; ++col) {
                m[r][col] = col == j ? t[r] : a[r][col];
            }
        }
        c[j] = det3(m) / det;
    }
    return c;
}

double evaluate(const Quadratic& c, const double x) {
    return c[0] + c[1] * x + c[2] * x * x;
}

double tangent_yaw(const Quadratic& c, const double x) {
    return std::atan(c[1] + 2.0 * c[2] * x);
}

std::vector<bool> ransac_inliers(
    const std::vector<Point2D>& points,
    const std::vector<double>&  weights,
    const double                threshold,
    const int                   iterations) {
    const std::size_t                          n = points.size();
    std::mt19937                               rng(0);
    std::uniform_int_distribution<std::size_t> pick(0, n - 1);
    const std::vector<double>                  unit(3, 1.0);
    std::vector<bool>                          best(n, true);
    double                                     best_score = 0.0;
    for (int it = 0; it < iterations; ++it) {
        const std::size_t a = pick(rng);
        const std::size_t b = pick(rng);
        const std::size_t c = pick(rng);
        if (a == b || b == c || a == c) {
            continue;
        }
        const Quadratic   model = fit_quadratic({points[a], points[b], points[c]}, unit);
        std::vector<bool> inliers(n);
        double            score = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            inliers[i] = std::abs(points[i].y - evaluate(model, points[i].x)) < threshold;
            if (inliers[i]) {
                score += weights[i];
            }
        }
        if (score > best_score) {
            best_score = score;
            best       = inliers;
        }
    }
    return best;
}

std::vector<Point2D> sample_quadratic(const Quadratic& c, const double x_start, const double x_end, const double interval) {
    std::vector<Point2D> points;
    // 弧長 interval を進むための x の刻みは interval·cos(yaw)。終点は x_end を超えない最後の刻み
    for (double x = x_start; x <= x_end;) {
        points.push_back(Point2D{x, evaluate(c, x)});
        const double yaw = tangent_yaw(c, x);
        if (std::abs(yaw) > MAX_TANGENT_RAD) {
            break;
        }
        x += interval * std::cos(yaw);
    }
    return points;
}

}  // namespace path_smoother
