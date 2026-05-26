#pragma once

#include <array>
#include <memory>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mpcc_controller/bounds.hpp"
#include "mpcc_controller/constraints.hpp"
#include "mpcc_controller/cost.hpp"
#include "mpcc_controller/hpipm_interface.hpp"
#include "mpcc_controller/integrator.hpp"
#include "mpcc_controller/motion_model.hpp"
#include "mpcc_controller/params.hpp"
#include "mpcc_controller/solver_interface.hpp"
#include "mpcc_controller/spline.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class MPC
{
public:
  MPC() = default;
  MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,
      std::shared_ptr<MotionModel> model,
      const Param & param, const CostParam & cost_param,
      const BoundsParam & bounds_param,
      const NormalizationParam & norm_param,
      double track_width_left, double track_width_right,
      const rclcpp::Logger & logger = rclcpp::get_logger("mpc"));

  MPCReturn runMPC(const State & x0);
  void setPath(const nav_msgs::msg::Path & path);
  double projectOnSpline(const State & x) const { return track_.projectOnSpline(x); }

private:
  void setMPCProblem();
  void setStage(const State & xk, const Input & uk, const State & xk1, int k);

  CostMatrix       normalizeCost(const CostMatrix & c)        const;
  LinModelMatrix   normalizeDynamics(const LinModelMatrix & m) const;
  ConstrainsMatrix normalizeCon(const ConstrainsMatrix & c)   const;
  std::array<OptVariables, N + 1> deNormalizeSolution(
    const std::array<OptVariables, N + 1> & sol) const;

  void updateInitialGuess(const State & x0);
  void generateNewInitialGuess(const State & x0);
  void unwrapInitialGuess();
  std::array<OptVariables, N + 1> sqpSolutionUpdate(
    const std::array<OptVariables, N + 1> & last,
    const std::array<OptVariables, N + 1> & curr) const;

  int    n_sqp_;
  int    n_reset_;
  double sqp_mixing_;
  double Ts_;

  bool valid_initial_guess_  = false;
  int  n_non_solves_         = 0;
  int  n_no_solves_sqp_      = 0;

  std::array<Stage, N + 1>        stages_;
  std::array<OptVariables, N + 1> initial_guess_;
  std::array<OptVariables, N + 1> optimal_solution_;

  std::shared_ptr<MotionModel>     model_;
  Integrator                       integrator_;
  Cost                             cost_;
  Constraints                      constraints_;
  Spline                           track_;
  Bounds                           bounds_;
  NormalizationParam               norm_param_;
  Param                            param_;
  std::unique_ptr<SolverInterface> solver_;

  double track_width_left_  = 2.0;
  double track_width_right_ = 2.0;

  rclcpp::Logger logger_{rclcpp::get_logger("mpc")};
};

}  // namespace mpcc_controller
