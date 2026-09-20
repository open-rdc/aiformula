#include "speed_path_planner/speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace speed_path_planner::speed_profile
{

namespace
{
constexpr double EPSILON = 1.0e-9;

double menger_curvature(const Point2D& p0, const Point2D& p1, const Point2D& p2)
{
    const double area2 =
        (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    const double a = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
    const double b = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const double c = std::hypot(p2[0] - p0[0], p2[1] - p0[1]);
    const double denom = a * b * c;
    if (denom < EPSILON) {
        return 0.0;
    }
    return 2.0 * area2 / denom;
}
}

std::vector<double> arc_lengths(const std::vector<Point2D>& points)
{
    std::vector<double> s(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        s[i] = s[i - 1] + std::hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
    }
    return s;
}

std::vector<double> curvatures(
    const std::vector<Point2D>& points, const std::vector<double>& s, double window_m)
{
    const std::size_t n = points.size();
    std::vector<double> kappa(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t lo = i;
        while (lo > 0 && s[i] - s[lo] < window_m) {
            --lo;
        }
        std::size_t hi = i;
        while (hi + 1 < n && s[hi] - s[i] < window_m) {
            ++hi;
        }
        kappa[i] = menger_curvature(points[lo], points[i], points[hi]);
    }
    kappa[0] = kappa[1];
    kappa[n - 1] = kappa[n - 2];
    return kappa;
}

std::vector<double> lateral_limits(const std::vector<double>& curvature, const Limits& limits)
{
    std::vector<double> v(curvature.size());
    for (std::size_t i = 0; i < curvature.size(); ++i) {
        v[i] = std::clamp(
            std::sqrt(limits.a_lat_max / std::abs(curvature[i])), limits.v_min, limits.v_max);
    }
    return v;
}

std::vector<double> apply_before_curve(
    const std::vector<double>& v, const std::vector<double>& s, double distance)
{
    std::vector<double> out(v);
    for (std::size_t i = 0; i < v.size(); ++i) {
        for (std::size_t j = i + 1; j < v.size() && s[j] - s[i] <= distance; ++j) {
            out[i] = std::min(out[i], v[j]);
        }
    }
    return out;
}

void apply_stop(std::vector<double>& v, const std::vector<double>& s, double stop_s)
{
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (s[i] >= stop_s) {
            v[i] = 0.0;
        }
    }
}

void backward_pass(std::vector<double>& v, const std::vector<double>& s, double a_lon)
{
    for (std::size_t i = v.size() - 1; i > 0; --i) {
        v[i - 1] = std::min(v[i - 1], std::sqrt(v[i] * v[i] + 2.0 * a_lon * (s[i] - s[i - 1])));
    }
}

void forward_pass(std::vector<double>& v, const std::vector<double>& s, double v_start, double a_lon)
{
    v[0] = std::min(v[0], v_start);
    for (std::size_t i = 1; i < v.size(); ++i) {
        v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * a_lon * (s[i] - s[i - 1])));
    }
}

std::vector<double> accelerations(const std::vector<double>& v, const std::vector<double>& s)
{
    const std::size_t n = v.size();
    std::vector<double> a(n, 0.0);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        a[i] = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * (s[i + 1] - s[i]));
    }
    a[n - 1] = a[n - 2];
    return a;
}

Profile plan(
    const std::vector<Point2D>& points, double v_meas, double stop_s, const Limits& limits)
{
    const auto s = arc_lengths(points);
    Profile profile;
    profile.curvature = curvatures(points, s, limits.curvature_window_m);
    profile.velocity = lateral_limits(profile.curvature, limits);
    profile.velocity = apply_before_curve(profile.velocity, s, limits.decel_distance_before_curve_m);
    apply_stop(profile.velocity, s, stop_s);
    backward_pass(profile.velocity, s, limits.a_lon);
    forward_pass(profile.velocity, s, std::max(v_meas, limits.v_min), limits.a_lon);
    profile.acceleration = accelerations(profile.velocity, s);
    return profile;
}

}
