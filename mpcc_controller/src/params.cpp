#include "mpcc_controller/params.hpp"

#include <vector>

namespace mpcc_controller {

namespace {

double get(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & p,
  const std::string & name)
{
  return p->get_parameter(name).as_double();
}

}  // namespace

Param loadParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix)
{
  Param p;
  p.vx_zero           = get(params, prefix + "vx_zero");
  p.max_dist_proj     = get(params, prefix + "max_dist_proj");
  p.s_trust_region    = get(params, prefix + "s_trust_region");
  p.initial_velocity  = get(params, prefix + "initial_velocity");
  p.car_l             = get(params, prefix + "car_l");
  p.car_w             = get(params, prefix + "car_w");
  p.max_lat_accel     = get(params, prefix + "max_lat_accel");
  p.v_max             = get(params, prefix + "v_max");
  p.lf                = get(params, prefix + "lf");
  p.lr                = get(params, prefix + "lr");
  p.wheelbase         = get(params, prefix + "wheelbase");
  return p;
}

CostParam loadCostParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix)
{
  CostParam c;
  c.q_c           = get(params, prefix + "q_c");
  c.q_l           = get(params, prefix + "q_l");
  c.q_vs          = get(params, prefix + "q_vs");
  c.q_mu          = get(params, prefix + "q_mu");
  c.q_mu_N_mult   = get(params, prefix + "q_mu_N_mult");
  c.q_c_N_mult    = get(params, prefix + "q_c_N_mult");
  c.q_r           = get(params, prefix + "q_r");
  c.q_r_N_mult    = get(params, prefix + "q_r_N_mult");
  c.r_v           = get(params, prefix + "r_v");
  c.r_omega       = get(params, prefix + "r_omega");
  c.r_vs          = get(params, prefix + "r_vs");
  c.r_dv          = get(params, prefix + "r_dv");
  c.r_domega      = get(params, prefix + "r_domega");
  c.r_dvs         = get(params, prefix + "r_dvs");
  c.r_vx          = get(params, prefix + "r_vx");
  c.r_delta       = get(params, prefix + "r_delta");
  c.r_a           = get(params, prefix + "r_a");
  c.r_ddelta      = get(params, prefix + "r_ddelta");
  c.sc_quad_track = get(params, prefix + "sc_quad_track");
  c.sc_lin_track  = get(params, prefix + "sc_lin_track");
  return c;
}

NormalizationParam loadNormalizationParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix)
{
  NormalizationParam n;

  const auto tx_diag = params->get_parameter(prefix + "T_x_diag").as_double_array();
  const auto tu_diag = params->get_parameter(prefix + "T_u_diag").as_double_array();
  const auto ts_diag = params->get_parameter(prefix + "T_s_diag").as_double_array();

  n.T_x     = TX_MPC::Zero();
  n.T_x_inv = TX_MPC::Zero();
  for (int i = 0; i < NX; i++) {
    n.T_x(i, i)     = tx_diag[i];
    n.T_x_inv(i, i) = 1.0 / tx_diag[i];
  }

  n.T_u     = TU_MPC::Zero();
  n.T_u_inv = TU_MPC::Zero();
  for (int i = 0; i < NU; i++) {
    n.T_u(i, i)     = tu_diag[i];
    n.T_u_inv(i, i) = 1.0 / tu_diag[i];
  }

  n.T_s     = TS_MPC::Zero();
  n.T_s_inv = TS_MPC::Zero();
  for (int i = 0; i < NS; i++) {
    n.T_s(i, i)     = ts_diag[i];
    n.T_s_inv(i, i) = 1.0 / ts_diag[i];
  }

  return n;
}

}  // namespace mpcc_controller
