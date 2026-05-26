#include "mpcc_controller/hpipm_interface.hpp"

#include <algorithm>
#include <cstdlib>

namespace mpcc_controller {

void HpipmInterface::setDynamics(std::array<Stage, N + 1> & stages, const State & x0)
{
  b0_ = stages[0].lin_model.A * stateToVector(x0) + stages[0].lin_model.g;

  for (int i = 0; i < N; i++) {
    if (i == 0) {
      hA_[i] = nullptr;
      hB_[i] = stages[i].lin_model.B.data();
      hb_[i] = b0_.data();
      nx_[i] = 0;
      nu_[i] = NU;
    } else {
      hA_[i] = stages[i].lin_model.A.data();
      hB_[i] = stages[i].lin_model.B.data();
      hb_[i] = stages[i].lin_model.g.data();
      nx_[i] = NX;
      nu_[i] = NU;
    }
  }
  nx_[N] = NX;
  nu_[N] = 0;
}

void HpipmInterface::setCost(std::array<Stage, N + 1> & stages)
{
  for (int i = 0; i <= N; i++) {
    hQ_[i] = stages[i].cost_mat.Q.data();
    hR_[i] = stages[i].cost_mat.R.data();
    hS_[i] = stages[i].cost_mat.S.data();
    hq_[i] = stages[i].cost_mat.q.data();
    hr_[i] = stages[i].cost_mat.r.data();

    if (stages[i].ns != 0) {
      hZl_[i] = stages[i].cost_mat.Z.data();
      hZu_[i] = stages[i].cost_mat.Z.data();
      hzl_[i] = stages[i].cost_mat.z.data();
      hzu_[i] = stages[i].cost_mat.z.data();
    } else {
      hZl_[i] = nullptr;
      hZu_[i] = nullptr;
      hzl_[i] = nullptr;
      hzu_[i] = nullptr;
    }
  }
}

void HpipmInterface::setBounds(std::array<Stage, N + 1> & stages, const State & /*x0*/)
{
  // stage 0: input bounds only
  nbu_[0] = 0;
  hpipm_bounds_[0].idx_u.clear();
  hpipm_bounds_[0].lower_bounds_u.clear();
  hpipm_bounds_[0].upper_bounds_u.clear();
  for (int j = 0; j < NU; j++) {
    if (stages[0].l_bounds_u(j) > -INF && stages[0].u_bounds_u(j) < INF) {
      nbu_[0]++;
      hpipm_bounds_[0].idx_u.push_back(j);
      hpipm_bounds_[0].lower_bounds_u.push_back(stages[0].l_bounds_u(j));
      hpipm_bounds_[0].upper_bounds_u.push_back(stages[0].u_bounds_u(j));
    }
  }
  nbx_[0]    = 0;
  hidxbx_[0] = nullptr;
  hidxbu_[0] = hpipm_bounds_[0].idx_u.data();
  hlbx_[0]   = nullptr;
  hubx_[0]   = nullptr;
  hlbu_[0]   = hpipm_bounds_[0].lower_bounds_u.data();
  hubu_[0]   = hpipm_bounds_[0].upper_bounds_u.data();

  for (int i = 1; i <= N; i++) {
    hpipm_bounds_[i].idx_u.clear();
    hpipm_bounds_[i].lower_bounds_u.clear();
    hpipm_bounds_[i].upper_bounds_u.clear();
    nbu_[i] = 0;
    for (int j = 0; j < NU; j++) {
      if (stages[i].l_bounds_u(j) > -INF && stages[i].u_bounds_u(j) < INF) {
        nbu_[i]++;
        hpipm_bounds_[i].idx_u.push_back(j);
        hpipm_bounds_[i].lower_bounds_u.push_back(stages[i].l_bounds_u(j));
        hpipm_bounds_[i].upper_bounds_u.push_back(stages[i].u_bounds_u(j));
      }
    }
    hpipm_bounds_[i].idx_x.clear();
    hpipm_bounds_[i].lower_bounds_x.clear();
    hpipm_bounds_[i].upper_bounds_x.clear();
    nbx_[i] = 0;
    for (int j = 0; j < NX; j++) {
      if (stages[i].l_bounds_x(j) > -INF && stages[i].u_bounds_x(j) < INF) {
        nbx_[i]++;
        hpipm_bounds_[i].idx_x.push_back(j);
        hpipm_bounds_[i].lower_bounds_x.push_back(stages[i].l_bounds_x(j));
        hpipm_bounds_[i].upper_bounds_x.push_back(stages[i].u_bounds_x(j));
      }
    }
    hidxbx_[i] = hpipm_bounds_[i].idx_x.data();
    hidxbu_[i] = hpipm_bounds_[i].idx_u.data();
    hlbx_[i]   = hpipm_bounds_[i].lower_bounds_x.data();
    hubx_[i]   = hpipm_bounds_[i].upper_bounds_x.data();
    hlbu_[i]   = hpipm_bounds_[i].lower_bounds_u.data();
    hubu_[i]   = hpipm_bounds_[i].upper_bounds_u.data();
  }

  nbu_[N]    = 0;
  hidxbu_[N] = nullptr;
  hlbu_[N]   = nullptr;
  hubu_[N]   = nullptr;
}

void HpipmInterface::setPolytopicConstraints(std::array<Stage, N + 1> & stages)
{
  for (int i = 0; i <= N; i++) {
    ng_[i] = stages[i].ng;
    if (stages[i].ng > 0) {
      hC_[i]  = stages[i].constrains_mat.C.data();
      hD_[i]  = stages[i].constrains_mat.D.data();
      hlg_[i] = stages[i].constrains_mat.dl.data();
      hug_[i] = stages[i].constrains_mat.du.data();
    } else {
      hC_[i] = hD_[i] = hlg_[i] = hug_[i] = nullptr;
    }
  }
}

void HpipmInterface::setSoftConstraints(std::array<Stage, N + 1> & stages)
{
  for (int i = 0; i <= N; i++) {
    hpipm_bounds_[i].idx_s.clear();
    if (stages[i].ns != 0) {
      ns_[i] = stages[i].ns;
      for (int j = 0; j < stages[i].ns; j++) {
        hpipm_bounds_[i].idx_s.push_back(j + nbx_[i] + nbu_[i]);
      }
      hidxs_[i] = hpipm_bounds_[i].idx_s.data();
      hlls_[i]  = stages[i].l_bounds_s.data();
      hlus_[i]  = stages[i].u_bounds_s.data();
    } else {
      ns_[i]    = 0;
      hidxs_[i] = nullptr;
      hlls_[i]  = nullptr;
      hlus_[i]  = nullptr;
    }
  }
}

std::array<OptVariables, N + 1> HpipmInterface::solveMPC(
  std::array<Stage, N + 1> & stages, const State & x0, int * status)
{
  setDynamics(stages, x0);
  setCost(stages);
  setBounds(stages, x0);
  setPolytopicConstraints(stages);
  setSoftConstraints(stages);

  auto opt = solve(status);
  opt[0].xk = x0;
  return opt;
}

std::array<OptVariables, N + 1> HpipmInterface::solve(int * status)
{
  // --- dim ---
  int dim_size = d_ocp_qp_dim_memsize(N);
  void * dim_mem = malloc(dim_size);
  struct d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(N, &dim, dim_mem);
  d_ocp_qp_dim_set_all(nx_, nu_, nbx_, nbu_, ng_, ns_, &dim);

  // --- qp ---
  int qp_size = d_ocp_qp_memsize(&dim);
  void * qp_mem = malloc(qp_size);
  struct d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, qp_mem);
  d_ocp_qp_set_all(
    hA_, hB_, hb_,
    hQ_, hS_, hR_, hq_, hr_,
    hidxbx_, hlbx_, hubx_,
    hidxbu_, hlbu_, hubu_,
    hC_, hD_, hlg_, hug_,
    hZl_, hZu_, hzl_, hzu_,
    hidxs_, nullptr, hlls_, hlus_, &qp);

  // --- qp sol ---
  int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
  void * qp_sol_mem = malloc(qp_sol_size);
  struct d_ocp_qp_sol qp_sol;
  d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

  // --- ipm arg ---
  int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
  void * ipm_arg_mem = malloc(ipm_arg_size);
  struct d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
  d_ocp_qp_ipm_arg_set_default(SPEED, &arg);
  int iter_max = 30;
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);

  // --- ipm workspace ---
  int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
  void * ipm_mem = malloc(ipm_size);
  struct d_ocp_qp_ipm_ws workspace;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

  // --- solve ---
  d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
  int hpipm_return;
  d_ocp_qp_ipm_get_status(&workspace, &hpipm_return);

  // --- extract solution ---
  const int nu_max = *std::max_element(nu_, nu_ + N + 1);
  const int nx_max = *std::max_element(nx_, nx_ + N + 1);
  double * u = reinterpret_cast<double *>(malloc(std::max(nu_max, 1) * sizeof(double)));
  double * x = reinterpret_cast<double *>(malloc(std::max(nx_max, 1) * sizeof(double)));

  std::array<OptVariables, N + 1> opt;
  opt[0].xk.setZero();
  for (int i = 1; i <= N; i++) {
    d_ocp_qp_sol_get_x(i, &qp_sol, x);
    opt[i].xk = arrayToState(x);
  }
  for (int i = 0; i < N; i++) {
    d_ocp_qp_sol_get_u(i, &qp_sol, u);
    opt[i].uk = arrayToInput(u);
  }
  opt[N].uk.setZero();

  free(u);
  free(x);
  free(dim_mem);
  free(qp_mem);
  free(qp_sol_mem);
  free(ipm_arg_mem);
  free(ipm_mem);

  *status = hpipm_return;
  return opt;
}

}  // namespace mpcc_controller
