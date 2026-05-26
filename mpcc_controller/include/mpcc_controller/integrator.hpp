#pragma once

#include "mpcc_controller/motion_model.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class Integrator
{
public:
  State RK4(const State & x, const Input & u, double ts,
            const MotionModel & model) const;
  State EF (const State & x, const Input & u, double ts,
            const MotionModel & model) const;
};

}  // namespace mpcc_controller
