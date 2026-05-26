#pragma once

#include <array>

#include "mpcc_controller/config.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class SolverInterface
{
public:
  virtual std::array<OptVariables, N + 1> solveMPC(
    std::array<Stage, N + 1> & stages,
    const State & x0,
    int * status) = 0;

  virtual ~SolverInterface() = default;
};

}  // namespace mpcc_controller
