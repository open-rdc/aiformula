#ifndef PATH_TRACKER__CASADI_SOLVER_HPP_
#define PATH_TRACKER__CASADI_SOLVER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void solve_mpc_casadi_c(
    const double *current_state, // 現在の状態 [5]: x, y, theta, v, delta
    const double *
        reference_trajectory, // 参照軌道 [horizon*4]: x, y, theta, v (繰り返し)
    int horizon, double dt, double wheelbase, double max_accel,
    double max_delta_rate,
    double *out_a,           // 出力: 最適加速度 [horizon]
    double *out_delta_rate); // 出力: 最適ステアリング速度 [horizon]

#ifdef __cplusplus
}
#endif

#endif // PATH_TRACKER__CASADI_SOLVER_HPP_
