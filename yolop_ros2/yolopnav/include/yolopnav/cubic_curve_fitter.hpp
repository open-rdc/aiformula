#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>
#include <limits>

namespace yolopnav {

struct CubicCurveCoefficients {
    double a, b, c, d;  // y = ax^3 + bx^2 + cx + d

    CubicCurveCoefficients() : a(0), b(0), c(0), d(0) {}
    CubicCurveCoefficients(double a_, double b_, double c_, double d_) : a(a_), b(b_), c(c_), d(d_) {}

    double evaluate(double x) const {
        return a * x * x * x + b * x * x + c * x + d;
    }
};

struct FittedCurve {
    CubicCurveCoefficients coefficients;
    std::vector<Eigen::Vector3d> fitted_points;
    std::vector<Eigen::Vector3d> inlier_points;
    int num_inliers;
    double score;

    FittedCurve() : num_inliers(0), score(0.0) {}
};

class CubicCurveFitter {
public:
    explicit CubicCurveFitter(int min_points_for_model = 4)
        : min_points_for_model_(min_points_for_model) {}

    ~CubicCurveFitter() = default;

    // Least squares cubic curve fitting for 3D points
    FittedCurve fitCubicCurve(const std::vector<Eigen::Vector3d>& points) const;

    // Generate uniformly spaced points along the fitted curve
    std::vector<Eigen::Vector3d> generateUniformPoints(const FittedCurve& curve,
                                                       double min_x, double max_x,
                                                       double interval = 0.5) const;

private:
    int min_points_for_model_;

    // Fit cubic curve using least squares
    CubicCurveCoefficients fitCubicToPoints(const std::vector<Eigen::Vector3d>& points) const;

    // Calculate distance from point to curve (for score calculation)
    double pointToCurveDistance(const Eigen::Vector3d& point, const CubicCurveCoefficients& curve) const;
};

}  // namespace yolopnav