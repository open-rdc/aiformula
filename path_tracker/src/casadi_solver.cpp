#include "path_tracker/casadi_solver.hpp"
#include <casadi/casadi.hpp>
#include <cmath>
#include <vector>

extern "C" {

void solve_mpc_casadi_c(const double *current_state,
                        const double *reference_trajectory, int horizon,
                        double dt, double wheelbase, double max_accel,
                        double max_delta_rate, double *out_a,
                        double *out_delta_rate) {
  using namespace casadi;

  // ソルバーを呼び出し間で持続させるためのスタティック変数
  static bool initialized = false;
  static Function solver;
  static int cached_N = -1;

  int N = horizon;
  int nx = 5; // 状態変数: x, y, theta, v, delta
  int nu = 2; // 制御入力: a, delta_rate

  static MX p_state = MX::sym("p_state", nx);
  static MX p_traj = MX::sym("p_traj", N * 4);

  if (!initialized || cached_N != N) {
    MX z = MX::sym("z", (N + 1) * nx + N * nu);
    MX obj = 0;
    MX g = MX::zeros((N + 1) * nx);

    // 重み設定 (安定性のために調整済み)
    double w_pos = 100.0;    // 位置追従の重み
    double w_yaw = 0.0;      // 向き（ヨー角）の重み (無効化)
    double w_vel = 0.0;      // 速度追従の重み (無効化)
    double w_accel = 0;      // 加速度へのペナルティ（振動抑制）
    double w_delta_rate = 0; // ステアリング速度へのペナルティ（ジッター抑制）

    // 初期状態の拘束
    g(Slice(0, nx)) = z(Slice(0, nx)) - p_state;

    for (int k = 0; k < N; ++k) {
      int st_idx = k * nx;
      int next_st_idx = (k + 1) * nx;
      int ctrl_idx = (N + 1) * nx + k * nu;

      MX x = z(Slice(st_idx, st_idx + nx));
      MX u = z(Slice(ctrl_idx, ctrl_idx + nu));
      MX x_next = z(Slice(next_st_idx, next_st_idx + nx));

      MX ref = p_traj(Slice(k * 4, (k + 1) * 4));

      MX dx = x(0) - ref(0);
      MX dy = x(1) - ref(1);
      MX dth = x(2) - ref(2);
      MX dv = x(3) - ref(3);

      // 目的関数: パス追従誤差と制御入力の最小化
      obj +=
          w_pos * (dx * dx + dy * dy) + w_yaw * (dth * dth) + w_vel * (dv * dv);
      obj += w_accel * (u(0) * u(0)) + w_delta_rate * (u(1) * u(1));

      // 運動学モデル（自転車モデル + 横滑り角考慮）の方程式
      // x_{k+1} = x_k + v_k * cos(theta_k + delta_k) * dt
      // y_{k+1} = y_k + v_k * sin(theta_k + delta_k) * dt
      MX x_next_model = MX::zeros(nx);
      x_next_model(0) = x(0) + x(3) * cos(x(2) + x(4)) * dt;
      x_next_model(1) = x(1) + x(3) * sin(x(2) + x(4)) * dt;
      x_next_model(2) = x(2) + (x(3) * tan(x(4)) / wheelbase) * dt;
      x_next_model(3) = x(3) + u(0) * dt;
      x_next_model(4) = x(4) + u(1) * dt;

      g(Slice(next_st_idx, next_st_idx + nx)) = x_next - x_next_model;
    }

    MXDict nlp = {
        {"x", z}, {"f", obj}, {"g", g}, {"p", MX::vertcat({p_state, p_traj})}};
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.max_iter"] = 100;

    solver = nlpsol("S", "ipopt", nlp, opts);
    cached_N = N;
    initialized = true;
  }

  // 入力引数の準備
  std::vector<double> p_val;
  for (int i = 0; i < nx; ++i)
    p_val.push_back(current_state[i]);
  for (int i = 0; i < N * 4; ++i)
    p_val.push_back(reference_trajectory[i]);

  // 変数の上下限設定 (拘束条件)
  std::vector<double> lbz((N + 1) * nx + N * nu, -inf);
  std::vector<double> ubz((N + 1) * nx + N * nu, inf);
  for (int k = 0; k < N; ++k) {
    int ctrl_idx = (N + 1) * nx + k * nu;
    lbz[ctrl_idx] = -max_accel;
    ubz[ctrl_idx] = max_accel;
    lbz[ctrl_idx + 1] = -max_delta_rate;
    ubz[ctrl_idx + 1] = max_delta_rate;
  }

  // ウォームスタート用の中間初期化
  std::vector<double> z0((N + 1) * nx + N * nu, 0.0);
  for (int k = 0; k <= N; ++k) {
    z0[k * nx + 3] = current_state[3];
    z0[k * nx + 4] = current_state[4];
  }

  DMDict arg = {{"x0", z0}, {"lbx", lbz}, {"ubx", ubz},
                {"lbg", 0}, {"ubg", 0},   {"p", p_val}};
  DMDict res = solver(arg);

  std::vector<double> sol_z = std::vector<double>(res.at("x"));

  // 最適な制御量を出力にコピー
  for (int k = 0; k < N; ++k) {
    int ctrl_idx = (N + 1) * nx + k * nu;
    out_a[k] = sol_z[ctrl_idx];
    out_delta_rate[k] = sol_z[ctrl_idx + 1];
  }
}

} // extern "C"
