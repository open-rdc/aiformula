#pragma once

#include "mpcc_controller/motion_model.hpp"
#include "mpcc_controller/params.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class Bounds
{
public:
  Bounds() = default;
  Bounds(const MotionModel & model, double s_trust_region);

  Bounds_x getBoundsLX(const State & x) const;
  Bounds_x getBoundsUX(const State & x) const;
  Bounds_x addVeloUX(const State & x, const Bounds_x & ub, double max_velo) const;
  Bounds_u getBoundsLU(const Input & u) const;
  Bounds_u getBoundsUU(const Input & u) const;
  Bounds_s getBoundsLS() const;
  Bounds_s getBoundsUS() const;

private:
  Bounds_x u_bounds_x_;
  Bounds_x l_bounds_x_;
  Bounds_u u_bounds_u_;
  Bounds_u l_bounds_u_;
  Bounds_s u_bounds_s_;
  Bounds_s l_bounds_s_;
  double   s_trust_region_ = 3.0;
};

}  // namespace mpcc_controller
