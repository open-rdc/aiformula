#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "mpcc_controller/config.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class MotionModel
{
public:
  virtual ~MotionModel() = default;

  virtual void initialize(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
    const std::string & plugin_name) = 0;

  // 連続時間ダイナミクス dx/dt = f(x, u)
  virtual StateVector getF(const State & x, const Input & u) const = 0;

  // x_next = A*x + B*u + g  (数値微分で離散化)
  // g = RK4(x_lin, u_lin) - x_next  (参照実装と同一)
  inline LinModelMatrix getLinModel(
    const State & x, const Input & u,
    const State & x_next, double Ts) const
  {
    return discretizeNumerical(x, u, Ts, stateToVector(x_next));
  }

  virtual Bounds_x getUpperBoundsX() const = 0;
  virtual Bounds_x getLowerBoundsX() const = 0;
  virtual Bounds_u getUpperBoundsU() const = 0;
  virtual Bounds_u getLowerBoundsU() const = 0;
  virtual Bounds_s getUpperBoundsS() const = 0;
  virtual Bounds_s getLowerBoundsS() const = 0;

protected:
  inline LinModelMatrix discretizeNumerical(
    const State & x, const Input & u, double Ts,
    const StateVector & x_next) const
  {
    constexpr double eps = 1e-5;

    const StateVector x_vec = stateToVector(x);
    const InputVector u_vec = inputToVector(u);

    auto rk4 = [&](const StateVector & xv, const InputVector & uv) -> StateVector {
      const State xs = vectorToState(xv);
      const Input us = vectorToInput(uv);
      const StateVector k1 = getF(xs, us);
      const StateVector k2 = getF(vectorToState(xv + Ts / 2.0 * k1), us);
      const StateVector k3 = getF(vectorToState(xv + Ts / 2.0 * k2), us);
      const StateVector k4 = getF(vectorToState(xv + Ts * k3), us);
      return xv + Ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
    };

    const StateVector x_next_nom = rk4(x_vec, u_vec);

    A_MPC A;
    for (int j = 0; j < NX; j++) {
      StateVector xp = x_vec, xm = x_vec;
      xp(j) += eps;
      xm(j) -= eps;
      A.col(j) = (rk4(xp, u_vec) - rk4(xm, u_vec)) / (2.0 * eps);
    }

    B_MPC B;
    for (int k = 0; k < NU; k++) {
      InputVector up = u_vec, um = u_vec;
      up(k) += eps;
      um(k) -= eps;
      B.col(k) = (rk4(x_vec, up) - rk4(x_vec, um)) / (2.0 * eps);
    }

    // 参照実装と同一: g = RK4(x_lin, u_lin) - x_next_lin
    const g_MPC g = x_next_nom - x_next;
    return {A, B, g};
  }
};


class DiffDriveMotionModel : public MotionModel
{
public:
  void initialize(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
    const std::string & plugin_name) override;

  StateVector getF(const State & x, const Input & u) const override;

  Bounds_x getUpperBoundsX() const override;
  Bounds_x getLowerBoundsX() const override;
  Bounds_u getUpperBoundsU() const override;
  Bounds_u getLowerBoundsU() const override;
  Bounds_s getUpperBoundsS() const override;
  Bounds_s getLowerBoundsS() const override;

private:
  double max_vel_;
  double max_omega_;
  double max_accel_;
  double max_alpha_;
  double max_dvs_;
};


class AckermannMotionModel : public MotionModel
{
public:
  void initialize(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params,
    const std::string & plugin_name) override;

  StateVector getF(const State & x, const Input & u) const override;

  Bounds_x getUpperBoundsX() const override;
  Bounds_x getLowerBoundsX() const override;
  Bounds_u getUpperBoundsU() const override;
  Bounds_u getLowerBoundsU() const override;
  Bounds_s getUpperBoundsS() const override;
  Bounds_s getLowerBoundsS() const override;

  float getMinTurningRadius() const { return min_turning_r_; }

private:
  double wheelbase_;
  float  min_turning_r_;
  double max_vx_;
  double max_delta_;
  double max_accel_;
  double max_ddelta_;
  double max_dvs_;
};

}  // namespace mpcc_controller
