#ifndef PATH_TRACKER__CASADI_SOLVER_HPP_
#define PATH_TRACKER__CASADI_SOLVER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void solve_mpc_casadi_c(
    const double *current_state, // [5]: x, y, theta, v, delta
    const double
        *reference_trajectory, // [horizon*4]: x, y, theta, v (repeated)
    int horizon, double dt, double wheelbase, double max_accel,
    double max_delta_rate,
    double *out_a,           // [horizon]
    double *out_delta_rate); // [horizon]

#ifdef __cplusplus
}
#endif

#endif // PATH_TRACKER__CASADI_SOLVER_HPP_
