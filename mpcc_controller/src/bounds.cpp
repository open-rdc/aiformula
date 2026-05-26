#include "mpcc_controller/bounds.hpp"

namespace mpcc_controller {

Bounds::Bounds(const MotionModel & model, const BoundsParam & param)
{
  u_bounds_x_ = model.getUpperBoundsX();
  l_bounds_x_ = model.getLowerBoundsX();

  // BoundsParam でオーバーライド (モデル依存パラメータと yaml 値を統合)
  u_bounds_x_(si_index.v)    = param.vx_u;
  l_bounds_x_(si_index.v)    = param.vx_l;
  u_bounds_x_(si_index.ctrl) = param.delta_u + param.omega_u;   // 一方が 0 のため加算でOK
  l_bounds_x_(si_index.ctrl) = param.delta_l + param.omega_l;
  u_bounds_x_(si_index.vs)   = param.vs_u;
  l_bounds_x_(si_index.vs)   = param.vs_l;

  u_bounds_u_ = model.getUpperBoundsU();
  l_bounds_u_ = model.getLowerBoundsU();

  u_bounds_u_(si_index.du0) = param.dv_u;
  l_bounds_u_(si_index.du0) = param.dv_l;
  u_bounds_u_(si_index.du1) = param.domega_u + param.ddelta_u;
  l_bounds_u_(si_index.du1) = param.domega_l + param.ddelta_l;
  u_bounds_u_(si_index.dvs) = param.dvs_u;
  l_bounds_u_(si_index.dvs) = param.dvs_l;

  l_bounds_s_ = Bounds_s::Zero();
  u_bounds_s_ = Bounds_s::Zero();
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
