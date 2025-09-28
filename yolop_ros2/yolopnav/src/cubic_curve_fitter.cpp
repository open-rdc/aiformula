#include "yolopnav/cubic_curve_fitter.hpp"
#include <cmath>
#include <limits>

namespace yolopnav {

FittedCurve CubicCurveFitter::fitCubicCurve(const std::vector<Eigen::Vector3d>& points) const {
    FittedCurve best_curve;

    if (points.size() < static_cast<size_t>(min_points_for_model_)) {
        return best_curve;
    }

    best_curve.score = std::numeric_limits<double>::max();

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        // Randomly sample 4 points
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rng_);

        std::vector<Eigen::Vector3d> sample_points;
        sample_points.reserve(min_points_for_model_);
        for (int i = 0; i < min_points_for_model_; ++i) {
            sample_points.push_back(points[indices[i]]);
        }

        // Fit cubic curve to these 4 points
        CubicCurveCoefficients curve_coeffs = fitCubicToPoints(sample_points);

        // Count inliers
        std::vector<Eigen::Vector3d> inliers = getInliers(points, curve_coeffs);

        if (inliers.size() >= static_cast<size_t>(min_points_for_model_)) {
            // Refit using all inliers for better accuracy
            CubicCurveCoefficients refined_coeffs = fitCubicToPoints(inliers);
            std::vector<Eigen::Vector3d> refined_inliers = getInliers(points, refined_coeffs);

            // Calculate score (lower is better) - using negative inlier count + error
            double total_error = 0.0;
            for (const auto& point : refined_inliers) {
                total_error += pointToCurveDistance(point, refined_coeffs);
            }
            double score = total_error / refined_inliers.size() - refined_inliers.size() * 0.1;

            if (score < best_curve.score || static_cast<size_t>(best_curve.num_inliers) < refined_inliers.size()) {
                best_curve.coefficients = refined_coeffs;
                best_curve.inlier_points = refined_inliers;
                best_curve.num_inliers = refined_inliers.size();
                best_curve.score = score;
            }
        }
    }

    return best_curve;
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
        uniform_points.emplace_back(x, y, 0.0);  // Assuming z=0 for lane on ground
    }

    return uniform_points;
}

CubicCurveCoefficients CubicCurveFitter::fitCubicToPoints(const std::vector<Eigen::Vector3d>& points) const {
    if (points.size() < static_cast<size_t>(min_points_for_model_)) {
        return CubicCurveCoefficients();
    }

    // Set up least squares system: A * coeffs = b
    // where coeffs = [a, b, c, d] for y = ax^3 + bx^2 + cx + d
    int n = points.size();
    Eigen::MatrixXd A(n, 4);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        double x = points[i].x();
        double y = points[i].y();

        A(i, 0) = x * x * x;  // x^3
        A(i, 1) = x * x;      // x^2
        A(i, 2) = x;          // x
        A(i, 3) = 1.0;        // constant

        b(i) = y;
    }

    // Solve using normal equations or SVD for robustness
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

    return CubicCurveCoefficients(coeffs(0), coeffs(1), coeffs(2), coeffs(3));
}

double CubicCurveFitter::pointToCurveDistance(const Eigen::Vector3d& point,
                                             const CubicCurveCoefficients& curve) const {
    double x = point.x();
    double y = point.y();
    double curve_y = curve.evaluate(x);

    // Simple vertical distance (could be improved with perpendicular distance)
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

}  // namespace yolopnav