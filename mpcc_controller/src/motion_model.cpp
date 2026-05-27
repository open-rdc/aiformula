#include "mpcc_controller/motion_model.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include "mpcc_controller/params.hpp"

namespace mpcc_controller {

// ============================================================
// DiffDriveMotionModel
// ============================================================

void DiffDriveMotionModel::initialize(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix)
{
  wheelbase_ = params->get_parameter(prefix + "wheelbase").as_double();
  v_zero_    = params->get_parameter(prefix + "vx_zero").as_double();
  max_vel_   = params->get_parameter(prefix + "max_vel").as_double();
  max_omega_ = params->get_parameter(prefix + "max_omega").as_double();
  max_accel_ = params->get_parameter(prefix + "max_accel").as_double();
  max_alpha_ = params->get_parameter(prefix + "max_alpha").as_double();
  max_dvs_   = params->get_parameter(prefix + "max_dvs").as_double();
}

StateVector DiffDriveMotionModel::getF(const State & x, const Input & u) const
{
  StateVector dx;
  dx(si_index.X)    = x.v_or_vx * std::cos(x.phi);
  dx(si_index.Y)    = x.v_or_vx * std::sin(x.phi);
  dx(si_index.phi)  = x.ctrl_state;   // φ̇ = omega_cmd
  dx(si_index.v)    = u.du0;           // v̇ = dv
  dx(si_index.ctrl) = u.du1;           // ω̇_cmd = domega
  dx(si_index.s)    = x.vs;
  dx(si_index.vs)   = u.dvs;
  return dx;
}

Bounds_x DiffDriveMotionModel::getUpperBoundsX() const
{
  Bounds_x ub;
  ub(si_index.X)    =  INF;
  ub(si_index.Y)    =  INF;
  ub(si_index.phi)  =  INF;
  ub(si_index.v)    =  max_vel_;
  ub(si_index.ctrl) =  max_omega_;
  ub(si_index.s)    =  INF;
  ub(si_index.vs)   =  max_vel_;
  return ub;
}

Bounds_x DiffDriveMotionModel::getLowerBoundsX() const
{
  Bounds_x lb;
  lb(si_index.X)    = -INF;
  lb(si_index.Y)    = -INF;
  lb(si_index.phi)  = -INF;
  lb(si_index.v)    =  0.0;
  lb(si_index.ctrl) = -max_omega_;
  lb(si_index.s)    = -INF;
  lb(si_index.vs)   =  0.0;
  return lb;
}

Bounds_u DiffDriveMotionModel::getUpperBoundsU() const
{
  Bounds_u ub;
  ub(si_index.du0) =  max_accel_;
  ub(si_index.du1) =  max_alpha_;
  ub(si_index.dvs) =  max_dvs_;
  return ub;
}

Bounds_u DiffDriveMotionModel::getLowerBoundsU() const
{
  Bounds_u lb;
  lb(si_index.du0) = -max_accel_;
  lb(si_index.du1) = -max_alpha_;
  lb(si_index.dvs) = -max_dvs_;
  return lb;
}

Bounds_s DiffDriveMotionModel::getUpperBoundsS() const { return Bounds_s::Zero(); }
Bounds_s DiffDriveMotionModel::getLowerBoundsS() const { return Bounds_s::Zero(); }

void DiffDriveMotionModel::applyInputCost(
  Q_MPC & Q_inp, R_MPC & R_inp, const CostParam & cp) const
{
  Q_inp(si_index.v,    si_index.v)    = cp.r_v;
  Q_inp(si_index.ctrl, si_index.ctrl) = cp.r_omega;
  Q_inp(si_index.vs,   si_index.vs)   = cp.r_vs;
  R_inp(si_index.du0, si_index.du0)   = cp.r_dv;
  R_inp(si_index.du1, si_index.du1)   = cp.r_domega;
  R_inp(si_index.dvs, si_index.dvs)   = cp.r_dvs;
}

void DiffDriveMotionModel::applyYawRateReg(Q_MPC & Q, double q_r) const
{
  // DiffDrive: ctrl_state = omega_cmd は yaw rate そのもの
  Q(si_index.ctrl, si_index.ctrl) += q_r;
}

double DiffDriveMotionModel::toSteeringAngle(double ctrl_state, double v_or_vx) const
{
  // omega = v * tan(delta) / L  ->  delta = atan(omega * L / v)
  // 低速時は v_zero で打ち切って発散を防ぐ
  const double v_eff = std::max(std::fabs(v_or_vx), v_zero_);
  return std::atan(ctrl_state * wheelbase_ / v_eff);
}

// ============================================================
// AckermannMotionModel
// ============================================================

void AckermannMotionModel::initialize(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix)
{
  wheelbase_      = params->get_parameter(prefix + "wheelbase").as_double();
  min_turning_r_  = static_cast<float>(params->get_parameter(prefix + "min_turning_r").as_double());
  max_vx_         = params->get_parameter(prefix + "max_vx").as_double();
  max_delta_      = params->get_parameter(prefix + "max_delta").as_double();
  max_accel_      = params->get_parameter(prefix + "max_accel").as_double();
  max_ddelta_     = params->get_parameter(prefix + "max_ddelta").as_double();
  max_dvs_        = params->get_parameter(prefix + "max_dvs").as_double();
}

StateVector AckermannMotionModel::getF(const State & x, const Input & u) const
{
  StateVector dx;
  dx(si_index.X)    = x.v_or_vx * std::cos(x.phi);
  dx(si_index.Y)    = x.v_or_vx * std::sin(x.phi);
  dx(si_index.phi)  = x.v_or_vx * std::tan(x.ctrl_state) / wheelbase_;
  dx(si_index.v)    = u.du0;   // v̇x = a
  dx(si_index.ctrl) = u.du1;   // δ̇ = ddelta
  dx(si_index.s)    = x.vs;
  dx(si_index.vs)   = u.dvs;
  return dx;
}

Bounds_x AckermannMotionModel::getUpperBoundsX() const
{
  Bounds_x ub;
  ub(si_index.X)    =  INF;
  ub(si_index.Y)    =  INF;
  ub(si_index.phi)  =  INF;
  ub(si_index.v)    =  max_vx_;
  ub(si_index.ctrl) =  max_delta_;
  ub(si_index.s)    =  INF;
  ub(si_index.vs)   =  max_vx_;
  return ub;
}

Bounds_x AckermannMotionModel::getLowerBoundsX() const
{
  Bounds_x lb;
  lb(si_index.X)    = -INF;
  lb(si_index.Y)    = -INF;
  lb(si_index.phi)  = -INF;
  lb(si_index.v)    =  0.0;
  lb(si_index.ctrl) = -max_delta_;
  lb(si_index.s)    = -INF;
  lb(si_index.vs)   =  0.0;
  return lb;
}

Bounds_u AckermannMotionModel::getUpperBoundsU() const
{
  Bounds_u ub;
  ub(si_index.du0) =  max_accel_;
  ub(si_index.du1) =  max_ddelta_;
  ub(si_index.dvs) =  max_dvs_;
  return ub;
}

Bounds_u AckermannMotionModel::getLowerBoundsU() const
{
  Bounds_u lb;
  lb(si_index.du0) = -max_accel_;
  lb(si_index.du1) = -max_ddelta_;
  lb(si_index.dvs) = -max_dvs_;
  return lb;
}

Bounds_s AckermannMotionModel::getUpperBoundsS() const { return Bounds_s::Zero(); }
Bounds_s AckermannMotionModel::getLowerBoundsS() const { return Bounds_s::Zero(); }

void AckermannMotionModel::applyInputCost(
  Q_MPC & Q_inp, R_MPC & R_inp, const CostParam & cp) const
{
  Q_inp(si_index.v,    si_index.v)    = cp.r_vx;
  Q_inp(si_index.ctrl, si_index.ctrl) = cp.r_delta;
  Q_inp(si_index.vs,   si_index.vs)   = cp.r_vs;
  R_inp(si_index.du0, si_index.du0)   = cp.r_a;
  R_inp(si_index.du1, si_index.du1)   = cp.r_ddelta;
  R_inp(si_index.dvs, si_index.dvs)   = cp.r_dvs;
}

void AckermannMotionModel::applyYawRateReg(Q_MPC & /*Q*/, double /*q_r*/) const
{
  // Ackermann: yaw rate = vx*tan(delta)/L であり ctrl_state(=delta) 単独では表せない。
  // 単純な線形ペナルティでは意味が変わるため，本キネマティクスでは適用しない
  // (dynamicモデル拡張時に yaw rate 状態を追加して効かせる想定)。
}

double AckermannMotionModel::toSteeringAngle(double ctrl_state, double /*v_or_vx*/) const
{
  return ctrl_state;
}

}  // namespace mpcc_controller

PLUGINLIB_EXPORT_CLASS(mpcc_controller::DiffDriveMotionModel, mpcc_controller::MotionModel)
PLUGINLIB_EXPORT_CLASS(mpcc_controller::AckermannMotionModel, mpcc_controller::MotionModel)
