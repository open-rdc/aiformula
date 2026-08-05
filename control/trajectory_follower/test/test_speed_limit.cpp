#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "trajectory_follower/mpc/speed_limit.hpp"

namespace tf = trajectory_follower;

TEST(SpeedLimit, MengerCurvatureIsZeroForColinearPoints)
{
    const std::array<double, 2> p0{0.0, 0.0};
    const std::array<double, 2> p1{1.0, 0.0};
    const std::array<double, 2> p2{2.0, 0.0};
    EXPECT_NEAR(tf::menger_curvature(p0, p1, p2), 0.0, 1e-9);
}

TEST(SpeedLimit, MengerCurvatureMatchesKnownRightAngleTurn)
{
    const std::array<double, 2> p0{0.0, 0.0};
    const std::array<double, 2> p1{1.0, 0.0};
    const std::array<double, 2> p2{1.0, 1.0};
    EXPECT_NEAR(tf::menger_curvature(p0, p1, p2), std::sqrt(2.0), 1e-6);
}

TEST(SpeedLimit, ForwardMaxCurvatureFindsSharpTurnWithinWindow)
{
    const std::vector<std::array<double, 2>> path_xy{
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {2.0, 2.0}};
    const std::vector<double> arc{0.0, 1.0, 2.0, 3.0, 4.0};
    EXPECT_NEAR(tf::forward_max_curvature(path_xy, arc, 0, 3.0), std::sqrt(2.0), 1e-6);
}

TEST(SpeedLimit, ForwardMaxCurvatureIgnoresTurnBeyondWindow)
{
    const std::vector<std::array<double, 2>> path_xy{
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {2.0, 2.0}};
    const std::vector<double> arc{0.0, 1.0, 2.0, 3.0, 4.0};
    EXPECT_NEAR(tf::forward_max_curvature(path_xy, arc, 0, 0.5), 0.0, 1e-9);
}

TEST(SpeedLimit, ComputeSpeedLimitCapsOnLateralAcceleration)
{
    const double result = tf::compute_speed_limit(10.0, 2.0, 2.0, 0.5, 1000.0);
    EXPECT_NEAR(result, 2.0, 1e-9);
}

TEST(SpeedLimit, ComputeSpeedLimitCapsOnBrakingDistance)
{
    const double result = tf::compute_speed_limit(10.0, 100.0, 2.0, 0.0, 2.0);
    EXPECT_NEAR(result, std::sqrt(8.0), 1e-6);
}

TEST(SpeedLimit, ComputeSpeedLimitReturnsVMaxWhenUnconstrained)
{
    const double result = tf::compute_speed_limit(3.0, 2.0, 2.0, 0.0, 1000.0);
    EXPECT_NEAR(result, 3.0, 1e-9);
}
