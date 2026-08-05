#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "trajectory_follower/mpc/lateral_mpc.hpp"

namespace tf = trajectory_follower;

namespace
{
// A path that curves sharply (quarter circle of the given radius, sampled every
// (pi/2)/steps radians) so the resulting steering command is far from zero. Used both to
// prove a tight steer_limit is actually clamping something non-trivial, and to prove the
// QP solve produces a curvature-direction-consistent, non-zero result.
//
// When with_lead_in is true, a straight segment from (0,0) to (x_offset,0) is prepended
// so the circle is tangent to, rather than starting exactly at, the vehicle's origin --
// this is what keeps e_y and e_yaw exactly zero while still exercising non-zero curvature.
std::vector<std::array<double, 2>> quarter_circle_path(
    double radius, double x_offset, int steps, bool with_lead_in)
{
    std::vector<std::array<double, 2>> path;
    if (with_lead_in) {
        path.push_back({0.0, 0.0});
    }
    for (int k = 0; k <= steps; ++k) {
        const double phi = static_cast<double>(k) / steps * (M_PI / 2.0);
        path.push_back({x_offset + radius * std::sin(phi), radius * (1.0 - std::cos(phi))});
    }
    return path;
}
}

TEST(LateralMpc, ReturnsClampedPreviousSteerWhenPathTooShort)
{
    // prev_steer_ starts at 0, so a path-too-short check against 0 can't distinguish a
    // correct clamp from a broken solver (both return 0). Instead, first drive prev_steer_
    // to a known, non-trivial clamped value using a sharply curving path and a very tight
    // steer_limit, then confirm the too-short-path fallback returns exactly that value.
    const auto curving_path =
        quarter_circle_path(/*radius=*/3.0, /*x_offset=*/0.0, /*steps=*/40, /*with_lead_in=*/false);

    // Sanity check: with an effectively unbounded steer_limit, the same path drives the
    // solver to a magnitude well beyond the 0.05 rad limit used below, so we know the
    // clamp exercised next is actually active (not a no-op).
    tf::LateralMpcParams unclamped_params{};
    unclamped_params.steer_limit = 10.0;
    tf::LateralMpc unclamped_mpc;
    unclamped_mpc.configure(unclamped_params);
    const double raw_steer = unclamped_mpc.computeSteering(curving_path, 1.0);
    ASSERT_GT(std::abs(raw_steer), 0.05);

    tf::LateralMpcParams params{};
    params.steer_limit = 0.05;
    tf::LateralMpc mpc;
    mpc.configure(params);
    const double clamped_steer = mpc.computeSteering(curving_path, 1.0);
    EXPECT_NEAR(std::abs(clamped_steer), 0.05, 1e-9);
    EXPECT_NEAR(mpc.previousSteer(), clamped_steer, 1e-9);

    const std::vector<std::array<double, 2>> too_short_path{{0.0, 0.0}, {1.0, 0.0}};
    const double result = mpc.computeSteering(too_short_path, 1.0);
    EXPECT_NEAR(result, clamped_steer, 1e-9);
    EXPECT_GT(std::abs(result), 1e-3);
}

TEST(LateralMpc, ProducesCurvatureConsistentNonzeroSteerOnConstantCurvaturePath)
{
    // Build a path where the vehicle sits exactly ON the path at the nearest point with
    // zero heading error (e_y = 0, e_yaw = 0): the first two points lie on the vehicle's
    // local x-axis (matching computeSteering's implicit vehicle pose of position (0,0),
    // heading 0), and every later point lies exactly on a circle that is tangent to that
    // x-axis segment. This keeps x0vec == 0 (no initial-state error to correct) while the
    // curvature disturbance term (Sw) is a known non-zero, constant-sign quantity, so a
    // genuinely working solve must still produce a non-zero, curvature-direction-consistent
    // steering command. A broken solve (e.g. NaN silently falling back to
    // prev_steer_ == 0) collapses to (near) zero and would fail the assertions below.
    //
    // We deliberately do not pin an exact expected magnitude (the closed-form bicycle
    // relationship is only approximate for this MPC's discretization/horizon), and instead
    // assert the sign/symmetry that must hold regardless of exact solver tuning: a
    // left-curving path steers one way, the mirrored right-curving path steers the exact
    // opposite way, and neither collapses to zero.
    const auto left_path =
        quarter_circle_path(/*radius=*/3.0, /*x_offset=*/0.1, /*steps=*/40, /*with_lead_in=*/true);
    std::vector<std::array<double, 2>> right_path = left_path;
    for (auto & p : right_path) {
        p[1] = -p[1];
    }

    tf::LateralMpc mpc_left;
    mpc_left.configure(tf::LateralMpcParams{});
    const double steer_left = mpc_left.computeSteering(left_path, 1.0);

    tf::LateralMpc mpc_right;
    mpc_right.configure(tf::LateralMpcParams{});
    const double steer_right = mpc_right.computeSteering(right_path, 1.0);

    EXPECT_GT(steer_left, 1e-3);
    EXPECT_LT(steer_right, -1e-3);
    EXPECT_NEAR(steer_left, -steer_right, 1e-9);
}

TEST(LateralMpc, ResetClearsPreviousSteerAndMeasuredSteer)
{
    tf::LateralMpc mpc;
    mpc.configure(tf::LateralMpcParams{});
    const std::vector<std::array<double, 2>> path_xy{{0.0, 0.0}, {1.0, 0.1}, {2.0, 0.2}};

    // Move prev_steer_ away from 0, then queue a measured-steer value that has not yet
    // been consumed by computeSteering, then reset(). reset() must clear both prev_steer_
    // and measured_steer_ -- if measured_steer_ were left set (the bug this guards
    // against), it would silently leak into the very next computeSteering call as the
    // initial steer state instead of the fresh (post-reset) prev_steer_ == 0.
    mpc.computeSteering(path_xy, 1.0);
    mpc.setMeasuredSteer(0.1);
    mpc.reset();
    EXPECT_NEAR(mpc.previousSteer(), 0.0, 1e-9);

    tf::LateralMpc fresh_mpc;
    fresh_mpc.configure(tf::LateralMpcParams{});

    const double reset_result = mpc.computeSteering(path_xy, 1.0);
    const double fresh_result = fresh_mpc.computeSteering(path_xy, 1.0);
    EXPECT_NEAR(reset_result, fresh_result, 1e-9);
}
