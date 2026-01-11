#pragma once

#include <vector>

namespace frenet_planner {

class ReferenceCurve {
public:
    ReferenceCurve(const std::vector<double>& x, const std::vector<double>& y);
    bool valid() const;
    double max_s() const;
    void sample(double s,
                double& x,
                double& y,
                double& yaw,
                double& kappa,
                double& dkappa) const;

private:
    std::vector<double> s_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    std::vector<double> kappa_;
    std::vector<double> dkappa_;
    bool valid_{false};
};

}  // namespace frenet_planner
