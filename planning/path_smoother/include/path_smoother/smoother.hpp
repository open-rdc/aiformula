#pragma once

#include "path_smoother/visibility_control.h"

#include <array>
#include <vector>

namespace path_smoother {

struct Point2D {
    double x;
    double y;
};

// base_link の odom 上の姿勢
struct Pose2D {
    double x;
    double y;
    double yaw;
};

// y = c[0] + c[1] x + c[2] x^2
using Quadratic = std::array<double, 3>;

PATH_SMOOTHER_PUBLIC
std::vector<Point2D> to_odom(const std::vector<Point2D>& points_base, const Pose2D& odom_T_base);

PATH_SMOOTHER_PUBLIC
std::vector<Point2D> to_base(const std::vector<Point2D>& points_odom, const Pose2D& odom_T_base);

// 各点の重み = 前後の半区間の和。端点は片側のみ。総和は折れ線の全長
PATH_SMOOTHER_PUBLIC
std::vector<double> arc_weights(const std::vector<Point2D>& points);

// 3 点で通した 2 次に対し |残差| < threshold の点の重み和を得点にする。シード固定で決定的
PATH_SMOOTHER_PUBLIC
std::vector<bool> ransac_inliers(
    const std::vector<Point2D>& points,
    const std::vector<double>&  weights,
    double                      threshold,
    int                         iterations);

PATH_SMOOTHER_PUBLIC
Quadratic fit_quadratic(const std::vector<Point2D>& points, const std::vector<double>& weights);

PATH_SMOOTHER_PUBLIC
double evaluate(const Quadratic& c, double x);

PATH_SMOOTHER_PUBLIC
double tangent_yaw(const Quadratic& c, double x);

// x_start から弧長 interval 刻みで x_end を超えない範囲まで展開する
PATH_SMOOTHER_PUBLIC
std::vector<Point2D> sample_quadratic(const Quadratic& c, double x_start, double x_end, double interval);

// start の次の点から x_to を超えない範囲まで yaw 方向に interval 刻みで直線を継ぐ
PATH_SMOOTHER_PUBLIC
std::vector<Point2D> extend_straight(const Point2D& start, double yaw, double x_to, double interval);

}  // namespace path_smoother
