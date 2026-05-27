#include "mpcc_controller/bounds.hpp"

namespace mpcc_controller {

Bounds::Bounds(const MotionModel & model, double s_trust_region)
: s_trust_region_(s_trust_region)
{
  // MotionModel が自身のパラメータ (max_vel/max_omega/max_delta 等) で
  // モデル固有の上下限を返すため、ここでは追加の上書きを行わない。
  u_bounds_x_ = model.getUpperBoundsX();
  l_bounds_x_ = model.getLowerBoundsX();
  u_bounds_u_ = model.getUpperBoundsU();
  l_bounds_u_ = model.getLowerBoundsU();
  u_bounds_s_ = model.getUpperBoundsS();
  l_bounds_s_ = model.getLowerBoundsS();
}

Bounds_x Bounds::getBoundsLX(const State & x) const
{
  Bounds_x lb = l_bounds_x_;
  lb(si_index.s) = x.s - s_trust_region_;
  return lb - stateToVector(x);
}

Bounds_x Bounds::getBoundsUX(const State & x) const
{
  Bounds_x ub = u_bounds_x_;
  ub(si_index.s) = x.s + s_trust_region_;
  return ub - stateToVector(x);
}

Bounds_x Bounds::addVeloUX(const State & x, const Bounds_x & ub, double max_velo) const
{
  Bounds_x mod = ub;
  mod(si_index.v) = max_velo - x.v_or_vx;
  return mod;
}

Bounds_u Bounds::getBoundsLU(const Input & u) const
{
  return l_bounds_u_ - inputToVector(u);
}

Bounds_u Bounds::getBoundsUU(const Input & u) const
{
  return u_bounds_u_ - inputToVector(u);
}

Bounds_s Bounds::getBoundsLS() const { return l_bounds_s_; }
Bounds_s Bounds::getBoundsUS() const { return u_bounds_s_; }

}  // namespace mpcc_controller
