#pragma once

#include <string>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "mpcc_controller/config.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

struct Param {
  double vx_zero;
  double max_dist_proj;
  double s_trust_region;
  double initial_velocity;
  double car_l;
  double car_w;
  double max_lat_accel;
  double v_max;
  // Ackermann のみ
  double lf;
  double lr;
  double wheelbase;
};

struct CostParam {
  double q_c;
  double q_l;
  double q_vs;
  double q_mu;
  double q_mu_N_mult;
  double q_c_N_mult;
  double q_r;
  double q_r_N_mult;
  // DiffDrive 入力コスト
  double r_v, r_omega, r_vs;
  double r_dv, r_domega, r_dvs;
  // Ackermann 入力コスト
  double r_vx, r_delta;
  double r_a, r_ddelta;
  // ソフト制約
  double sc_quad_track, sc_lin_track;
};

struct NormalizationParam {
  TX_MPC T_x,   T_x_inv;
  TU_MPC T_u,   T_u_inv;
  TS_MPC T_s,   T_s_inv;
};

// ROS 2 パラメータから各パラメータ構造体を生成する関数
Param             loadParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix);

CostParam         loadCostParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix);

NormalizationParam loadNormalizationParam(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
  const std::string & prefix);

}  // namespace mpcc_controller
