#include <gtest/gtest.h>

#include <cmath>

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
    tf::LongitudinalPid pid;
    pid.configure(make_params());
    // With make_params() (v_max=5.0, a_lat_max=2.0, a_min=-2.0), referenceSpeed(0.5, 100.0):
    //   lateral term  = sqrt(a_lat_max / kappa)          = sqrt(2.0 / 0.5)   = 2.0
    //   braking term  = sqrt(2 * |a_min| * dist_to_end)  = sqrt(2*2.0*100.0) = 20.0
    //   result        = clamp(min(2.0, 20.0), 0, v_max)  = 2.0
    EXPECT_NEAR(pid.referenceSpeed(0.5, 100.0), 2.0, 1e-9);
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
