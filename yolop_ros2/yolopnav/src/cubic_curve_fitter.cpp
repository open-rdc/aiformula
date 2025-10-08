#include "yolopnav/cubic_curve_fitter.hpp"
#include <cmath>
#include <limits>

namespace yolopnav {

// 最小二乗法による3次曲線フィッティング（外れ値除外付き）
FittedCurve CubicCurveFitter::fitCubicCurve(const std::vector<Eigen::Vector3d>& points) const {
    FittedCurve result;

    if (points.size() < static_cast<size_t>(min_points_for_model_)) {
        return result;
    }

    CubicCurveCoefficients initial_curve = fitCubicToPoints(points);
    std::vector<Eigen::Vector3d> inliers = getInliers(points, initial_curve);
    if (inliers.size() < static_cast<size_t>(min_points_for_model_)) {
        return result;
    }

    // 再フィッティング: 外れ値除外後の点群で再度3次曲線をフィッティング
    CubicCurveCoefficients refined_curve = fitCubicToPoints(inliers);

    result.coefficients = refined_curve;
    result.inlier_points = inliers;
    result.num_inliers = inliers.size();

    double total_error = 0.0;
    for (const auto& point : inliers) {
        total_error += std::pow(pointToCurveDistance(point, refined_curve), 2);
    }
    result.score = total_error / inliers.size();

    return result;
}

CubicCurveCoefficients CubicCurveFitter::fitCubicToPoints(const std::vector<Eigen::Vector3d>& points) const {

    int n = points.size();
    Eigen::MatrixXd A(n, 4);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].x();
        double y = points[i].y();

        A(i, 0) = x * x * x;  // x³
        A(i, 1) = x * x;      // x²
        A(i, 2) = x;          // x
        A(i, 3) = 1.0;        // 定数項

        b(i) = y;
    }

    // QR分解による連立方程式の求解
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

    return CubicCurveCoefficients(coeffs(0), coeffs(1), coeffs(2), coeffs(3));
}

double CubicCurveFitter::pointToCurveDistance(const Eigen::Vector3d& point,
                                             const CubicCurveCoefficients& curve) const {
    double x = point.x();
    double y = point.y();
    double curve_y = curve.evaluate(x);

    // 垂直方向の距離（y方向の差）
    return std::abs(y - curve_y);
}

std::vector<Eigen::Vector3d> CubicCurveFitter::getInliers(const std::vector<Eigen::Vector3d>& points,
                                                          const CubicCurveCoefficients& curve) const {
    std::vector<Eigen::Vector3d> inliers;

    for (const auto& point : points) {
        if (pointToCurveDistance(point, curve) < inlier_threshold_) {
            inliers.push_back(point);
        }
    }

    return inliers;
}

std::vector<Eigen::Vector3d> CubicCurveFitter::generateUniformPoints(const FittedCurve& curve,
                                                                     double min_x, double max_x,
                                                                     double interval) const {
    std::vector<Eigen::Vector3d> uniform_points;

    if (curve.num_inliers == 0) {
        return uniform_points;
    }

    for (double x = min_x; x <= max_x; x += interval) {
        double y = curve.coefficients.evaluate(x);
        uniform_points.emplace_back(x, y, 0.0);  // z=0 (地面上のレーン)
    }

    return uniform_points;
}

}  // namespace yolopnav