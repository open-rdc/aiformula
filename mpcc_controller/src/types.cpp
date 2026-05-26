#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

StateVector stateToVector(const State & x)
{
  StateVector xk;
  xk(si_index.X)    = x.X;
  xk(si_index.Y)    = x.Y;
  xk(si_index.phi)  = x.phi;
  xk(si_index.v)    = x.v_or_vx;
  xk(si_index.ctrl) = x.ctrl_state;
  xk(si_index.s)    = x.s;
  xk(si_index.vs)   = x.vs;
  return xk;
}

InputVector inputToVector(const Input & u)
{
  InputVector uk;
  uk(si_index.du0) = u.du0;
  uk(si_index.du1) = u.du1;
  uk(si_index.dvs) = u.dvs;
  return uk;
}

State vectorToState(const StateVector & v)
{
  State x;
  x.X          = v(si_index.X);
  x.Y          = v(si_index.Y);
  x.phi        = v(si_index.phi);
  x.v_or_vx    = v(si_index.v);
  x.ctrl_state = v(si_index.ctrl);
  x.s          = v(si_index.s);
  x.vs         = v(si_index.vs);
  return x;
}

Input vectorToInput(const InputVector & v)
{
  Input u;
  u.du0 = v(si_index.du0);
  u.du1 = v(si_index.du1);
  u.dvs = v(si_index.dvs);
  return u;
}

State arrayToState(const double * xk)
{
  State x;
  x.X          = xk[si_index.X];
  x.Y          = xk[si_index.Y];
  x.phi        = xk[si_index.phi];
  x.v_or_vx    = xk[si_index.v];
  x.ctrl_state = xk[si_index.ctrl];
  x.s          = xk[si_index.s];
  x.vs         = xk[si_index.vs];
  return x;
}

Input arrayToInput(const double * uk)
{
  Input u;
  u.du0 = uk[si_index.du0];
  u.du1 = uk[si_index.du1];
  u.dvs = uk[si_index.dvs];
  return u;
}

}  // namespace mpcc_controller
