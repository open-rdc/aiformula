#include "mpcc_controller/constraints.hpp"

#include <cmath>

namespace mpcc_controller {

Constraints::Constraints(const Param & param)
: param_(param) {}

OneDConstraint Constraints::getTrackConstraints(
  const Spline & track, const State & x) const
{
  const double s = x.s;

  const Eigen::Vector2d pos_center = track.getPosition(s);
  const Eigen::Vector2d d_center   = track.getDerivative(s);

  // 法線ベクトル (経路接線に直交)
  const Eigen::Vector2d normal      = {-d_center(1), d_center(0)};
  const double          path_heading = std::atan2(d_center(1), d_center(0));

  const double n_left  = track.getNLeft(s);
  const double n_right = track.getNRight(s);

  const double heading_error = std::atan2(
    std::sin(x.phi - path_heading), std::cos(x.phi - path_heading));
  const double cos_mu = std::cos(heading_error);
  const double sin_mu = std::sin(heading_error);

  const double projected_size   =
    param_.car_w * cos_mu + param_.car_l * std::fabs(sin_mu);
  const double d_projected_size =
    -param_.car_w * sin_mu + param_.car_l * cos_mu * (heading_error > 0.0 ? 1.0 : -1.0);

  const Eigen::Vector2d pos_outer = pos_center + (n_left  - 0.5 * projected_size) * normal;
  const Eigen::Vector2d pos_inner = pos_center + (n_right + 0.5 * projected_size) * normal;

  const Eigen::Vector2d car_pos   = {x.X, x.Y};
  const double          cur_proj  = normal.dot(car_pos);

  const double dl = normal.dot(pos_inner) - cur_proj;
  const double du = normal.dot(pos_outer) - cur_proj;

  C_i_MPC C_track = C_i_MPC::Zero();
  C_track(0, si_index.X)   = normal(0);
  C_track(0, si_index.Y)   = normal(1);
  if (std::fabs(dl) < std::fabs(du)) {
    C_track(0, si_index.phi) = -0.5 * d_projected_size;
  } else {
    C_track(0, si_index.phi) =  0.5 * d_projected_size;
  }

  return {C_track, dl, du};
}

ConstrainsMatrix Constraints::getConstraints(
  const Spline & track, const State & x, const Input & /*u*/) const
{
  const OneDConstraint tc = getTrackConstraints(track, x);

  C_MPC C = C_MPC::Zero();
  D_MPC D = D_MPC::Zero();
  d_MPC dl, du;

  C.row(si_index.con_track) = tc.C_i;
  dl(si_index.con_track)    = tc.dl_i;
  du(si_index.con_track)    = tc.du_i;

  return {C, D, dl, du};
}

}  // namespace mpcc_controller
