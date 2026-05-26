#include "mpcc_controller/integrator.hpp"

namespace mpcc_controller {

State Integrator::RK4(
  const State & x, const Input & u, double ts,
  const MotionModel & model) const
{
  const StateVector x_vec = stateToVector(x);
  const StateVector k1 = model.getF(x, u);
  const StateVector k2 = model.getF(vectorToState(x_vec + ts / 2.0 * k1), u);
  const StateVector k3 = model.getF(vectorToState(x_vec + ts / 2.0 * k2), u);
  const StateVector k4 = model.getF(vectorToState(x_vec + ts * k3), u);
  return vectorToState(x_vec + ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0));
}

State Integrator::EF(
  const State & x, const Input & u, double ts,
  const MotionModel & model) const
{
  return vectorToState(stateToVector(x) + ts * model.getF(x, u));
}

}  // namespace mpcc_controller
