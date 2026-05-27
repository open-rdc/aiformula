#include "mpcc_controller/mpc.hpp"

#include <algorithm>
#include <chrono>

namespace mpcc_controller {

MPC::MPC(
  int n_sqp, int n_reset, double sqp_mixing, double Ts,
  std::shared_ptr<MotionModel> model,
  const Param & param, const CostParam & cost_param,
  const NormalizationParam & norm_param,
  double track_width_left, double track_width_right,
  const rclcpp::Logger & logger)
: n_sqp_(n_sqp),
  n_reset_(n_reset),
  sqp_mixing_(sqp_mixing),
  Ts_(Ts),
  model_(model),
  cost_(cost_param, param, model),
  constraints_(param),
  bounds_(*model, param.s_trust_region),
  norm_param_(norm_param),
  param_(param),
  solver_(std::make_unique<HpipmInterface>()),
  track_width_left_(track_width_left),
  track_width_right_(track_width_right),
  logger_(logger)
{}

void MPC::setPath(const nav_msgs::msg::Path & path)
{
  track_.genSplines(path, param_, track_width_left_, track_width_right_);
}

void MPC::setMPCProblem()
{
  for (int i = 0; i <= N; i++) {
    setStage(
      initial_guess_[i].xk, initial_guess_[i].uk,
      initial_guess_[std::min(i + 1, N)].xk, i);
  }
}

void MPC::setStage(const State & xk, const Input & uk, const State & xk1, int k)
{
  stages_[k].nx = NX;
  stages_[k].nu = NU;

  if (k == 0) {
    stages_[k].ng = 0;
    stages_[k].ns = 0;
  } else {
    stages_[k].ng = NPC;
    stages_[k].ns = NS;
  }

  State xk_nz  = xk;   xk_nz.enforceMinSpeed(param_.vx_zero);
  State xk1_nz = xk1;  xk1_nz.enforceMinSpeed(param_.vx_zero);

  stages_[k].cost_mat      = normalizeCost(cost_.getCost(track_, xk_nz, uk, k));
  stages_[k].lin_model     = normalizeDynamics(
    model_->getLinModel(xk_nz, uk, xk1_nz, Ts_));
  stages_[k].constrains_mat = normalizeCon(
    constraints_.getConstraints(track_, xk_nz, uk));

  stages_[k].l_bounds_x = norm_param_.T_x_inv * bounds_.getBoundsLX(xk_nz);
  if (k == N) {
    const double terminal_v = track_.getVelocity(xk_nz.s);
    stages_[k].u_bounds_x = norm_param_.T_x_inv *
      bounds_.addVeloUX(xk_nz, bounds_.getBoundsUX(xk_nz), terminal_v);
  } else {
    stages_[k].u_bounds_x = norm_param_.T_x_inv * bounds_.getBoundsUX(xk_nz);
  }
  stages_[k].l_bounds_u = norm_param_.T_u_inv * bounds_.getBoundsLU(uk);
  stages_[k].u_bounds_u = norm_param_.T_u_inv * bounds_.getBoundsUU(uk);
  stages_[k].l_bounds_s = norm_param_.T_s_inv * bounds_.getBoundsLS();
  stages_[k].u_bounds_s = norm_param_.T_s_inv * bounds_.getBoundsUS();
}

CostMatrix MPC::normalizeCost(const CostMatrix & c) const
{
  return {
    norm_param_.T_x * c.Q * norm_param_.T_x,
    norm_param_.T_u * c.R * norm_param_.T_u,
    S_MPC::Zero(),
    norm_param_.T_x * c.q,
    norm_param_.T_u * c.r,
    norm_param_.T_s * c.Z * norm_param_.T_s,
    norm_param_.T_s * c.z
  };
}

LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix & m) const
{
  return {
    norm_param_.T_x_inv * m.A * norm_param_.T_x,
    norm_param_.T_x_inv * m.B * norm_param_.T_u,
    norm_param_.T_x_inv * m.g
  };
}

ConstrainsMatrix MPC::normalizeCon(const ConstrainsMatrix & c) const
{
  return {
    c.C * norm_param_.T_x,
    c.D * norm_param_.T_u,
    c.dl,
    c.du
  };
}

std::array<OptVariables, N + 1> MPC::deNormalizeSolution(
  const std::array<OptVariables, N + 1> & sol) const
{
  std::array<OptVariables, N + 1> out;
  for (int i = 0; i <= N; i++) {
    out[i].xk = vectorToState(norm_param_.T_x * stateToVector(sol[i].xk));
    out[i].uk = vectorToInput(norm_param_.T_u * inputToVector(sol[i].uk));
  }
  return out;
}

void MPC::updateInitialGuess(const State & x0)
{
  // 1 ステップ分シフトしてウォームスタートを維持する。
  // 直前解の u[1..N-1] が新しい ig[0..N-2].uk に，状態も同様にシフトされる。
  for (int i = 1; i < N; i++) initial_guess_[i - 1] = initial_guess_[i];
  initial_guess_[0].xk = x0;
  // 末端は (N-2) の制御入力を踏襲して終端ホライズンを定速進行させる
  initial_guess_[N - 1].xk = initial_guess_[N - 2].xk;
  initial_guess_[N - 1].uk = initial_guess_[N - 2].uk;
  initial_guess_[N].xk = integrator_.RK4(
    initial_guess_[N - 1].xk, initial_guess_[N - 1].uk, Ts_, *model_);
  initial_guess_[N].uk.setZero();
  unwrapInitialGuess();
}

void MPC::generateNewInitialGuess(const State & x0)
{
  initial_guess_[0].xk = x0;
  initial_guess_[0].uk.setZero();

  for (int i = 1; i <= N; i++) {
    initial_guess_[i].xk.setZero();
    initial_guess_[i].uk.setZero();
    initial_guess_[i].xk.s   = initial_guess_[i - 1].xk.s + Ts_ * param_.initial_velocity;
    const Eigen::Vector2d pos = track_.getPosition(initial_guess_[i].xk.s);
    const Eigen::Vector2d dpos = track_.getDerivative(initial_guess_[i].xk.s);
    initial_guess_[i].xk.X        = pos(0);
    initial_guess_[i].xk.Y        = pos(1);
    initial_guess_[i].xk.phi      = std::atan2(dpos(1), dpos(0));
    initial_guess_[i].xk.v_or_vx  = param_.initial_velocity;
    initial_guess_[i].xk.vs       = param_.initial_velocity;
  }
  unwrapInitialGuess();
  valid_initial_guess_ = true;
}

void MPC::unwrapInitialGuess()
{
  const double L = track_.getLength();
  const bool   closed = track_.isClosed();
  for (int i = 1; i <= N; i++) {
    const double dphi = initial_guess_[i].xk.phi - initial_guess_[i - 1].xk.phi;
    if (dphi < -PI) initial_guess_[i].xk.phi += 2.0 * PI;
    if (dphi >  PI) initial_guess_[i].xk.phi -= 2.0 * PI;

    // 閉ループのときのみ s の周回ジャンプを補正
    if (closed && (initial_guess_[i].xk.s - initial_guess_[i - 1].xk.s) > L / 2.0) {
      initial_guess_[i].xk.s -= L;
    }
  }
}

std::array<OptVariables, N + 1> MPC::sqpSolutionUpdate(
  const std::array<OptVariables, N + 1> & last,
  const std::array<OptVariables, N + 1> & curr) const
{
  // HPIPM はデルタ形式: curr[i].xk/uk は x_lin からの物理デルタ。
  // SQP 更新: new = last + α * Δ
  std::array<OptVariables, N + 1> out;
  for (int i = 0; i <= N; i++) {
    out[i].xk = vectorToState(
      stateToVector(last[i].xk) + sqp_mixing_ * stateToVector(curr[i].xk));
    out[i].uk = vectorToInput(
      inputToVector(last[i].uk) + sqp_mixing_ * inputToVector(curr[i].uk));
  }
  return out;
}

MPCReturn MPC::runMPC(const State & x0)
{
  const auto t1 = std::chrono::high_resolution_clock::now();

  State x_mpc = x0;
  x_mpc.unwrapPhi();
  x_mpc.s = track_.wrapOrClampS(x_mpc.s);

  const bool used_new_guess = !valid_initial_guess_;
  if (valid_initial_guess_) updateInitialGuess(x_mpc);
  else                      generateNewInitialGuess(x_mpc);

  RCLCPP_DEBUG(logger_,
    "[runMPC] start: valid_ig=%d, x0=(X=%.3f,Y=%.3f,phi=%.3f,v=%.3f,s=%.3f,vs=%.3f)",
    !used_new_guess,
    x_mpc.X, x_mpc.Y, x_mpc.phi, x_mpc.v_or_vx, x_mpc.s, x_mpc.vs);

  n_no_solves_sqp_ = 0;
  for (int i = 0; i < n_sqp_; i++) {
    setMPCProblem();
    State x0_norm;
    x0_norm.setZero();
    int solver_status = -1;
    optimal_solution_ = solver_->solveMPC(stages_, x0_norm, &solver_status);
    optimal_solution_ = deNormalizeSolution(optimal_solution_);
    if (solver_status != 0) n_no_solves_sqp_++;
    if (solver_status <= 1) {
      initial_guess_ = sqpSolutionUpdate(initial_guess_, optimal_solution_);
    }
    RCLCPP_DEBUG(logger_,
      "[runMPC] SQP[%d] status=%d, opt[1].(v=%.4f,vs=%.4f,s=%.3f)",
      i, solver_status,
      optimal_solution_[1].xk.v_or_vx,
      optimal_solution_[1].xk.vs,
      optimal_solution_[1].xk.s);
  }

  const int max_error = std::max(n_sqp_ - 1, 1);
  if (n_no_solves_sqp_ >= max_error) n_non_solves_++;
  else                               n_non_solves_ = 0;

  if (n_non_solves_ >= n_reset_) {
    RCLCPP_WARN(logger_,
      "[runMPC] initial_guess reset (n_non_solves=%d >= n_reset=%d)",
      n_non_solves_, n_reset_);
    valid_initial_guess_ = false;
    n_non_solves_ = 0;
  }

  const auto t2 = std::chrono::high_resolution_clock::now();
  const double time_total =
    std::chrono::duration<double>(t2 - t1).count();

  RCLCPP_INFO(logger_,
    "[runMPC] new_guess=%d, no_solves=%d/%d, non_solves=%d | "
    "u0=(du0=%.4f,du1=%.4f,dvs=%.4f) | "
    "ig[1]=(v=%.4f,vs=%.4f,ctrl=%.4f,s=%.3f) | time=%.1fms",
    used_new_guess, n_no_solves_sqp_, n_sqp_, n_non_solves_,
    initial_guess_[0].uk.du0, initial_guess_[0].uk.du1, initial_guess_[0].uk.dvs,
    initial_guess_[1].xk.v_or_vx, initial_guess_[1].xk.vs,
    initial_guess_[1].xk.ctrl_state, initial_guess_[1].xk.s,
    time_total * 1000.0);

  return {initial_guess_[0].uk, initial_guess_, time_total, used_new_guess};
}

}  // namespace mpcc_controller
