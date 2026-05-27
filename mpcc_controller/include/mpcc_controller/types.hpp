#pragma once

#include <cmath>

#include "mpcc_controller/config.hpp"

namespace mpcc_controller {

// 行列型エイリアス (固定サイズ Eigen 型)
using StateVector = Eigen::Matrix<double, NX, 1>;
using InputVector = Eigen::Matrix<double, NU, 1>;
using A_MPC       = Eigen::Matrix<double, NX, NX>;
using B_MPC       = Eigen::Matrix<double, NX, NU>;
using g_MPC       = Eigen::Matrix<double, NX,  1>;
using Q_MPC       = Eigen::Matrix<double, NX, NX>;
using R_MPC       = Eigen::Matrix<double, NU, NU>;
using S_MPC       = Eigen::Matrix<double, NX, NU>;
using q_MPC       = Eigen::Matrix<double, NX,  1>;
using r_MPC       = Eigen::Matrix<double, NU,  1>;
using C_MPC       = Eigen::Matrix<double, NPC, NX>;
using C_i_MPC     = Eigen::Matrix<double, 1,   NX>;
using D_MPC       = Eigen::Matrix<double, NPC, NU>;
using d_MPC       = Eigen::Matrix<double, NPC,  1>;
using Z_MPC       = Eigen::Matrix<double, NS,  NS>;
using z_MPC       = Eigen::Matrix<double, NS,   1>;
using Bounds_x    = Eigen::Matrix<double, NX,   1>;
using Bounds_u    = Eigen::Matrix<double, NU,   1>;
using Bounds_s    = Eigen::Matrix<double, NS,   1>;
using TX_MPC      = Eigen::Matrix<double, NX,  NX>;
using TU_MPC      = Eigen::Matrix<double, NU,  NU>;
using TS_MPC      = Eigen::Matrix<double, NS,  NS>;

struct State {
  double X        = 0.0;
  double Y        = 0.0;
  double phi      = 0.0;
  double v_or_vx  = 0.0;   // DiffDrive: v,  Ackermann: vx
  double ctrl_state = 0.0; // DiffDrive: omega_cmd,  Ackermann: delta
  double s        = 0.0;
  double vs       = 0.0;

  void setZero() {
    X = Y = phi = v_or_vx = ctrl_state = s = vs = 0.0;
  }

  // phi のみ ±π に正規化。s の周回処理はトラックの開/閉ループに依存するため Spline 側で行う
  void unwrapPhi() {
    while (phi >  PI) phi -= 2.0 * PI;
    while (phi < -PI) phi += 2.0 * PI;
  }

  void enforceMinSpeed(double v_min) {
    if (v_or_vx < v_min) v_or_vx = v_min;
  }
};

struct Input {
  double du0  = 0.0;   // DiffDrive: dv,  Ackermann: a
  double du1  = 0.0;   // DiffDrive: domega,  Ackermann: ddelta
  double dvs  = 0.0;

  void setZero() { du0 = du1 = dvs = 0.0; }
};

struct LinModelMatrix {
  A_MPC A;
  B_MPC B;
  g_MPC g;
};

struct CostMatrix {
  Q_MPC Q;
  R_MPC R;
  S_MPC S;
  q_MPC q;
  r_MPC r;
  Z_MPC Z;
  z_MPC z;
};

struct ConstrainsMatrix {
  C_MPC C;
  D_MPC D;
  d_MPC dl;
  d_MPC du;
};

struct OptVariables {
  State xk;
  Input uk;
};

struct Stage {
  LinModelMatrix    lin_model;
  CostMatrix        cost_mat;
  ConstrainsMatrix  constrains_mat;
  Bounds_x u_bounds_x, l_bounds_x;
  Bounds_u u_bounds_u, l_bounds_u;
  Bounds_s u_bounds_s, l_bounds_s;
  int nx, nu, nbx, nbu, ng, ns;
};

struct MPCReturn {
  const Input                             u0;
  const std::array<OptVariables, N + 1>  mpc_horizon;
  const double                            time_total;
  const bool                              guess_reset;  // 今回のループで initial_guess を再生成したか
};

// フリー関数 (si_index を使って State/Input ↔ Eigen ベクトル変換)
StateVector stateToVector(const State & x);
InputVector inputToVector(const Input & u);
State       vectorToState(const StateVector & v);
Input       vectorToInput(const InputVector & v);
State       arrayToState(const double * xk);
Input       arrayToInput(const double * uk);

}  // namespace mpcc_controller
