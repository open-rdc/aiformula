#include "path_smoother/smoother.hpp"

#include <cmath>
#include <random>

namespace path_smoother {

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

Cubic fit_cubic(const std::vector<Point2D>& points, const std::vector<double>& weights) {
    // 正規方程式 (Σ w x^(i+j)) c = Σ w y x^i をガウスの消去法で解く
    double s[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double t[4] = {0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < points.size(); ++i) {
        double xk = 1.0;
        for (int k = 0; k < 7; ++k) {
            s[k] += weights[i] * xk;
            if (k < 4) {
                t[k] += weights[i] * points[i].y * xk;
            }
            xk *= points[i].x;
        }
    }
    double a[4][5] = {{s[0], s[1], s[2], s[3], t[0]}, {s[1], s[2], s[3], s[4], t[1]}, {s[2], s[3], s[4], s[5], t[2]}, {s[3], s[4], s[5], s[6], t[3]}};
    for (int p = 0; p < 4; ++p) {
        for (int r = p + 1; r < 4; ++r) {
            const double f = a[r][p] / a[p][p];
            for (int col = p; col < 5; ++col) {
                a[r][col] -= f * a[p][col];
            }
        }
    }
    Cubic c{};
    for (int r = 3; r >= 0; --r) {
        c[r] = a[r][4];
        for (int col = r + 1; col < 4; ++col) {
            c[r] -= a[r][col] * c[col];
        }
        c[r] /= a[r][r];
    }
    return c;
}

double evaluate(const Cubic& c, const double x) {
    return c[0] + x * (c[1] + x * (c[2] + x * c[3]));
}

double tangent_yaw(const Cubic& c, const double x) {
    return std::atan(c[1] + x * (2.0 * c[2] + x * 3.0 * c[3]));
}

std::vector<bool> ransac_inliers(
    const std::vector<Point2D>& points,
    const std::vector<double>&  weights,
    const double                threshold,
    const int                   iterations) {
    const std::size_t                          n = points.size();
    std::mt19937                               rng(0);
    std::uniform_int_distribution<std::size_t> pick(0, n - 1);
    const std::vector<double>                  unit(4, 1.0);
    std::vector<bool>                          best(n, true);
    double                                     best_score = 0.0;
    for (int it = 0; it < iterations; ++it) {
        const std::size_t a = pick(rng);
        const std::size_t b = pick(rng);
        const std::size_t c = pick(rng);
        const std::size_t d = pick(rng);
        if (a == b || a == c || a == d || b == c || b == d || c == d) {
            continue;
        }
        const Cubic       model = fit_cubic({points[a], points[b], points[c], points[d]}, unit);
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

std::vector<Point2D> sample_cubic(const Cubic& c, const double x_start, const double x_end, const double interval) {
    std::vector<Point2D> points;
    // 弧長 interval を進むための x の刻みは interval·cos(yaw)。終点は x_end を超えない最後の刻み
    for (double x = x_start; x <= x_end;) {
        points.push_back(Point2D{x, evaluate(c, x)});
        const double yaw = tangent_yaw(c, x);
        x += interval * std::cos(yaw);
    }
    return points;
}

}  // namespace path_smoother
