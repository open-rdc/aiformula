#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "trajectory_follower/mpc/lateral_mpc.hpp"

namespace tf = trajectory_follower;

TEST(LateralMpc, ReturnsClampedPreviousSteerWhenPathTooShort)
{
    tf::LateralMpc mpc;
    mpc.configure(tf::LateralMpcParams{});
    const std::vector<std::array<double, 2>> path_xy{{0.0, 0.0}, {1.0, 0.0}};
    EXPECT_NEAR(mpc.computeSteering(path_xy, 1.0), 0.0, 1e-9);
}

TEST(LateralMpc, ReturnsZeroSteerOnPerfectlyStraightPath)
{
    tf::LateralMpc mpc;
    mpc.configure(tf::LateralMpcParams{});
    std::vector<std::array<double, 2>> path_xy;
    for (int i = 0; i <= 40; ++i) {
        path_xy.push_back({static_cast<double>(i) * 0.5, 0.0});
    }
    EXPECT_NEAR(mpc.computeSteering(path_xy, 1.0), 0.0, 1e-9);
}

TEST(LateralMpc, ResetReturnsPreviousSteerToZero)
{
    tf::LateralMpc mpc;
    mpc.configure(tf::LateralMpcParams{});
    mpc.setMeasuredSteer(0.1);
    std::vector<std::array<double, 2>> path_xy{{0.0, 0.0}, {1.0, 0.1}, {2.0, 0.2}};
    mpc.computeSteering(path_xy, 1.0);
    mpc.reset();
    EXPECT_NEAR(mpc.previousSteer(), 0.0, 1e-9);
}
