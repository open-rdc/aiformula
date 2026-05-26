#include "mpcc_controller/cost.hpp"

#include <cmath>

namespace mpcc_controller {

Cost::Cost(const CostParam & cost_param, const Param & param)
: cost_param_(cost_param), param_(param) {}

TrackPoint Cost::getRefPoint(const Spline & track, const State & x) const
{
  const double s = x.s;
  const Eigen::Vector2d pos   = track.getPosition(s);
  const Eigen::Vector2d dpos  = track.getDerivative(s);
  const Eigen::Vector2d ddpos = track.getSecondDerivative(s);

  const double dx = dpos(0), dy = dpos(1);
  const double ddx = ddpos(0), ddy = ddpos(1);
  const double theta_ref = std::atan2(dy, dx);

  double dtheta_nom   = dx * ddy - dy * ddx;
  double dtheta_denom = dx * dx + dy * dy;
  if (std::fabs(dtheta_nom)   < 1e-7) dtheta_nom   = 0.0;
  if (std::fabs(dtheta_denom) < 1e-7) dtheta_denom = 1e-7;

  return {pos(0), pos(1), dx, dy, theta_ref, dtheta_nom / dtheta_denom};
}

ErrorInfo Cost::getErrorInfo(const Spline & track, const State & x) const
{
  const TrackPoint tp = getRefPoint(track, x);
  const double X = x.X, Y = x.Y;

  Eigen::Matrix<double, 1, 2> error;
  error(0) = -std::sin(tp.theta_ref) * (tp.x_ref - X) + std::cos(tp.theta_ref) * (tp.y_ref - Y);
  error(1) =  std::cos(tp.theta_ref) * (tp.x_ref - X) + std::sin(tp.theta_ref) * (tp.y_ref - Y);

  const double dContErr =
    -tp.dtheta_ref * std::cos(tp.theta_ref) * (tp.x_ref - X)
    - tp.dtheta_ref * std::sin(tp.theta_ref) * (tp.y_ref - Y)
    - tp.dx_ref * std::sin(tp.theta_ref)
    + tp.dy_ref * std::cos(tp.theta_ref);

  const double dLagErr =
    -tp.dtheta_ref * std::sin(tp.theta_ref) * (tp.x_ref - X)
    + tp.dtheta_ref * std::cos(tp.theta_ref) * (tp.y_ref - Y)
    + tp.dx_ref * std::cos(tp.theta_ref)
    + tp.dy_ref * std::sin(tp.theta_ref);

  Eigen::Matrix<double, 2, NX> d_error = Eigen::Matrix<double, 2, NX>::Zero();
  d_error(0, si_index.X) =  std::sin(tp.theta_ref);
  d_error(0, si_index.Y) = -std::cos(tp.theta_ref);
  d_error(0, si_index.s) =  dContErr;
  d_error(1, si_index.X) = -std::cos(tp.theta_ref);
  d_error(1, si_index.Y) = -std::sin(tp.theta_ref);
  d_error(1, si_index.s) =  dLagErr;

  return {error, d_error};
}

CostMatrix Cost::getContouringCost(const Spline & track, const State & x, int k) const
{
  const ErrorInfo ei = getErrorInfo(track, x);
  const StateVector x_vec = stateToVector(x);

  const double q_c = (k < N) ? cost_param_.q_c : cost_param_.q_c_N_mult * cost_param_.q_c;
  const double q_l = cost_param_.q_l;

  Eigen::Matrix<double, 1, NX> d_cont = ei.d_error.row(0);
  Eigen::Matrix<double, 1, NX> d_lag  = ei.d_error.row(1);

  const double e_cont0 = ei.error(0) - d_cont * x_vec;
  const double e_lag0  = ei.error(1) - d_lag  * x_vec;

  Q_MPC Q_cont = q_c * d_cont.transpose() * d_cont + q_l * d_lag.transpose() * d_lag;
  // ヨーレート正則化 (DiffDrive: ctrl_state = omega_cmd)
  const double q_r = (k < N) ? cost_param_.q_r : cost_param_.q_r_N_mult * cost_param_.q_r;
  Q_cont(si_index.ctrl, si_index.ctrl) += q_r;
  Q_cont = 2.0 * Q_cont;

  q_MPC q_cont = q_c * 2.0 * e_cont0 * d_cont.transpose()
               + q_l * 2.0 * e_lag0  * d_lag.transpose();
  q_cont(si_index.vs) = -cost_param_.q_vs;

  return {Q_cont, R_MPC::Zero(), S_MPC::Zero(), q_cont,
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getHeadingCost(const Spline & track, const State & x, int k) const
{
  const Eigen::Vector2d dpos = track.getDerivative(x.s);
  double theta_ref = std::atan2(dpos(1), dpos(0));
  theta_ref += 2.0 * PI * std::round((x.phi - theta_ref) / (2.0 * PI));

  const double q_mu = (k < N)
    ? cost_param_.q_mu
    : cost_param_.q_mu_N_mult * cost_param_.q_mu;

  Q_MPC Q_h = Q_MPC::Zero();
  Q_h(si_index.phi, si_index.phi) = 2.0 * q_mu;
  q_MPC q_h = q_MPC::Zero();
  q_h(si_index.phi) = -2.0 * q_mu * theta_ref;

  return {Q_h, R_MPC::Zero(), S_MPC::Zero(), q_h,
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getInputCost() const
{
  Q_MPC Q_inp = Q_MPC::Zero();
  R_MPC R_inp = R_MPC::Zero();

  // DiffDrive と Ackermann は si_index.v / ctrl が共通レイアウト
  // DiffDrive: v, omega_cmd / Ackermann: vx, delta
  Q_inp(si_index.v,    si_index.v)    = cost_param_.r_v    + cost_param_.r_vx;
  Q_inp(si_index.ctrl, si_index.ctrl) = cost_param_.r_omega + cost_param_.r_delta;
  Q_inp(si_index.vs,   si_index.vs)   = cost_param_.r_vs;

  R_inp(si_index.du0, si_index.du0) = cost_param_.r_dv  + cost_param_.r_a;
  R_inp(si_index.du1, si_index.du1) = cost_param_.r_domega + cost_param_.r_ddelta;
  R_inp(si_index.dvs, si_index.dvs) = cost_param_.r_dvs;

  Q_inp = 2.0 * Q_inp;
  R_inp = 2.0 * R_inp;

  return {Q_inp, R_inp, S_MPC::Zero(), q_MPC::Zero(),
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getSoftConstraintCost() const
{
  Z_MPC Z = Z_MPC::Zero();
  z_MPC z = z_MPC::Zero();
  Z(si_index.con_track, si_index.con_track) = cost_param_.sc_quad_track;
  z(si_index.con_track)                     = cost_param_.sc_lin_track;
  return {Q_MPC::Zero(), R_MPC::Zero(), S_MPC::Zero(), q_MPC::Zero(),
          r_MPC::Zero(), Z, z};
}

CostMatrix Cost::getCost(
  const Spline & track, const State & x, const Input & u, int k) const
{
  const CostMatrix cc = getContouringCost(track, x, k);
  const CostMatrix hc = getHeadingCost(track, x, k);
  const CostMatrix ic = getInputCost();
  const CostMatrix sc = getSoftConstraintCost();

  const Q_MPC Q_ns = cc.Q + hc.Q + ic.Q;
  const Q_MPC Q = 0.5 * (Q_ns.transpose() + Q_ns);
  const R_MPC R = cc.R + ic.R;
  const q_MPC q = cc.q + hc.q + ic.q + (stateToVector(x).adjoint() * Q).adjoint();
  const r_MPC r = cc.r + ic.r + (inputToVector(u).adjoint() * R).adjoint();
  const Z_MPC Z = 2.0 * sc.Z;
  const z_MPC z = sc.z;

  return {Q, R, S_MPC::Zero(), q, r, Z, z};
}

}  // namespace mpcc_controller
