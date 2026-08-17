#pragma once

namespace local_planner::frenet
{

class Polynomial
{
public:
    Polynomial(
        double x0, double x0v, double x0a,
        double xT, double xTv, double xTa,
        double T);

    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double jerk(double t) const;

private:
    double c0_;
    double c1_;
    double c2_;
    double c3_;
    double c4_;
    double c5_;
};

}
