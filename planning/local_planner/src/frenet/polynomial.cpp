#include "local_planner/frenet/polynomial.hpp"

#include <algorithm>

namespace local_planner::frenet
{
namespace
{
constexpr double MIN_DURATION = 1.0e-9;
}

Polynomial::Polynomial(
    const double x0, const double x0v, const double x0a,
    const double xT, const double xTv, const double xTa,
    const double T)
: c0_(x0), c1_(x0v), c2_(0.5 * x0a)
{
    const double duration = std::max(T, MIN_DURATION);
    const double T2 = duration * duration;
    const double T3 = T2 * duration;
    const double T4 = T3 * duration;
    const double T5 = T4 * duration;
    const double dp = xT - x0 - x0v * duration - 0.5 * x0a * T2;
    const double dv = xTv - x0v - x0a * duration;
    const double da = xTa - x0a;
    c3_ = (10.0 * dp - 4.0 * dv * duration + 0.5 * da * T2) / T3;
    c4_ = (-15.0 * dp + 7.0 * dv * duration - da * T2) / T4;
    c5_ = (6.0 * dp - 3.0 * dv * duration + 0.5 * da * T2) / T5;
}

double Polynomial::position(const double t) const
{
    return c0_ + t * (c1_ + t * (c2_ + t * (c3_ + t * (c4_ + t * c5_))));
}

double Polynomial::velocity(const double t) const
{
    return c1_ + t * (2.0 * c2_ + t * (3.0 * c3_ + t * (4.0 * c4_ + t * 5.0 * c5_)));
}

double Polynomial::acceleration(const double t) const
{
    return 2.0 * c2_ + t * (6.0 * c3_ + t * (12.0 * c4_ + t * 20.0 * c5_));
}

double Polynomial::jerk(const double t) const
{
    return 6.0 * c3_ + t * (24.0 * c4_ + t * 60.0 * c5_);
}

}
