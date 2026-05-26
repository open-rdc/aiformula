#pragma once

#include "mpcc_controller/params.hpp"
#include "mpcc_controller/spline.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

struct TrackPoint {
  double x_ref, y_ref;
  double dx_ref, dy_ref;
  double theta_ref;
  double dtheta_ref;
};

struct ErrorInfo {
  Eigen::Matrix<double, 1, 2>  error;
  Eigen::Matrix<double, 2, NX> d_error;
};

class Cost
{
public:
  Cost() = default;
  Cost(const CostParam & cost_param, const Param & param);

  CostMatrix getCost(
    const Spline & track, const State & x, const Input & u, int k) const;

private:
  TrackPoint getRefPoint(const Spline & track, const State & x) const;
  ErrorInfo   getErrorInfo(const Spline & track, const State & x) const;

  CostMatrix getContouringCost(const Spline & track, const State & x, int k) const;
  CostMatrix getHeadingCost(const Spline & track, const State & x, int k) const;
  CostMatrix getInputCost() const;
  CostMatrix getSoftConstraintCost() const;

  CostParam cost_param_;
  Param     param_;
};

}  // namespace mpcc_controller
