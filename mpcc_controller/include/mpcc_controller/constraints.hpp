#pragma once

#include "mpcc_controller/params.hpp"
#include "mpcc_controller/spline.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

struct OneDConstraint {
  C_i_MPC C_i;
  double  dl_i;
  double  du_i;
};

class Constraints
{
public:
  Constraints() = default;
  Constraints(const Param & param);

  ConstrainsMatrix getConstraints(
    const Spline & track, const State & x, const Input & u) const;

private:
  OneDConstraint getTrackConstraints(const Spline & track, const State & x) const;

  Param param_;
};

}  // namespace mpcc_controller
