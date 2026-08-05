#include <gtest/gtest.h>

#include "trajectory_follower/mpc/longitudinal_pid.hpp"
#include "trajectory_follower/mpc/speed_limit.hpp"

namespace tf = trajectory_follower;

namespace
{
tf::LongitudinalParams make_params()
{
    tf::LongitudinalParams params;
    params.v_max = 5.0;
    params.a_lat_max = 2.0;
    params.a_max = 1.0;
    params.a_min = -2.0;
    params.jerk_max = 2.0;
    params.kp = 0.8;
    params.ki = 0.1;
    params.kd = 0.0;
    params.lpf_vel_error_gain = 0.9;
    params.dt = 0.05;
    return params;
}
}

TEST(LongitudinalPid, ReturnsZeroWhenStoppedAndReferenceIsStop)
{
    tf::LongitudinalPid pid;
    pid.configure(make_params());
    EXPECT_NEAR(pid.update(0.0, 0.0), 0.0, 1e-9);
}

TEST(LongitudinalPid, FirstCallWithZeroErrorHoldsMeasuredSpeed)
{
    tf::LongitudinalPid pid;
    pid.configure(make_params());
    EXPECT_NEAR(pid.update(2.0, 2.0), 2.0, 1e-9);
}

TEST(LongitudinalPid, ReferenceSpeedMatchesSpeedLimitFunction)
{
    const auto params = make_params();
    tf::LongitudinalPid pid;
    pid.configure(params);
    const double expected = tf::compute_speed_limit(params.v_max, params.a_lat_max, std::abs(params.a_min), 0.5, 100.0);
    EXPECT_NEAR(pid.referenceSpeed(0.5, 100.0), expected, 1e-9);
}

TEST(LongitudinalPid, ResetClearsInternalStateBackToFirstCallBehavior)
{
    tf::LongitudinalPid pid;
    pid.configure(make_params());
    pid.update(3.0, 0.0);
    pid.update(3.0, 1.0);
    pid.reset();
    EXPECT_NEAR(pid.update(2.0, 2.0), 2.0, 1e-9);
}
