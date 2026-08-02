#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "local_planner/frenet/frenet_planner.hpp"
#include "local_planner/frenet/hard_constraint.hpp"
#include "local_planner/frenet/polynomial.hpp"
#include "local_planner/frenet/soft_constraint.hpp"

namespace lf = local_planner::frenet;

namespace
{
constexpr double QUINTIC_PEAK_ACCEL_COEFF = 5.773502691896258;

std::vector<double> make_s_grid(const double start_s, const double end_s, const double step)
{
    std::vector<double> grid;
    for (double s = start_s; s < end_s; s += step) {
        grid.push_back(s);
    }
    grid.push_back(end_s);
    return grid;
}
}

TEST(Polynomial, BoundaryConditions)
{
    const lf::Polynomial poly(0.3, -0.2, 0.1, 1.2, 0.05, -0.02, 6.0);
    EXPECT_NEAR(poly.position(0.0), 0.3, 1e-9);
    EXPECT_NEAR(poly.velocity(0.0), -0.2, 1e-9);
    EXPECT_NEAR(poly.acceleration(0.0), 0.1, 1e-9);
    EXPECT_NEAR(poly.position(6.0), 1.2, 1e-9);
    EXPECT_NEAR(poly.velocity(6.0), 0.05, 1e-9);
    EXPECT_NEAR(poly.acceleration(6.0), -0.02, 1e-9);
}

TEST(Polynomial, MatchesQuinticSmoothstep)
{
    const lf::Polynomial poly(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
    for (const double t : {0.1, 0.25, 0.5, 0.75, 0.9}) {
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        EXPECT_NEAR(poly.position(t), 6.0 * t5 - 15.0 * t4 + 10.0 * t3, 1e-12);
        EXPECT_NEAR(poly.velocity(t), 30.0 * t4 - 60.0 * t3 + 30.0 * t2, 1e-12);
        EXPECT_NEAR(poly.acceleration(t), 120.0 * t3 - 180.0 * t2 + 60.0 * t, 1e-12);
    }
}

TEST(Polynomial, PeakAccelerationMatchesCoefficient)
{
    const double shift = 0.8;
    const double length = 4.0;
    const lf::Polynomial poly(0.0, 0.0, 0.0, shift, 0.0, 0.0, length);
    double max_accel = 0.0;
    for (double t = 0.0; t <= length; t += 1e-4) {
        max_accel = std::max(max_accel, std::abs(poly.acceleration(t)));
    }
    EXPECT_NEAR(max_accel, QUINTIC_PEAK_ACCEL_COEFF * shift / (length * length), 1e-4);
}

TEST(GenerateCandidate, SamplesQuinticBetweenStates)
{
    const lf::FrenetState initial{10.0, 0.5, -0.1, 0.02};
    const lf::FrenetState target{25.0, -0.3, 0.0, 0.0};
    const auto candidate = lf::generate_candidate(initial, target, 0.5);
    ASSERT_FALSE(candidate.s_points.empty());
    ASSERT_EQ(candidate.s_points.size(), candidate.offsets.size());
    EXPECT_NEAR(candidate.s_points.front(), 10.5, 1e-9);
    EXPECT_NEAR(candidate.s_points.back(), 25.0, 1e-9);
    ASSERT_TRUE(candidate.lateral_polynomial.has_value());
    const lf::Polynomial expected(0.5, -0.1, 0.02, -0.3, 0.0, 0.0, 15.0);
    for (std::size_t i = 0U; i < candidate.s_points.size(); ++i) {
        EXPECT_NEAR(
            candidate.offsets[i],
            expected.position(candidate.s_points[i] - 10.0), 1e-9);
    }
    EXPECT_NEAR(candidate.offsets.back(), -0.3, 1e-9);
}

TEST(GenerateCandidate, ReachesTargetWithZeroSlope)
{
    const lf::FrenetState initial{0.0, 0.4, 0.0, 0.0};
    const lf::FrenetState target{15.0, 0.0, 0.0, 0.0};
    const auto candidate = lf::generate_candidate(initial, target, 0.2);
    ASSERT_GE(candidate.offsets.size(), 2U);
    EXPECT_NEAR(candidate.offsets.back(), 0.0, 1e-9);
    const std::size_t n = candidate.offsets.size();
    EXPECT_NEAR(candidate.offsets[n - 1U] - candidate.offsets[n - 2U], 0.0, 1e-3);
}

TEST(GenerateCandidate, EmptyWhenTargetBehindInitial)
{
    const lf::FrenetState initial{10.0, 0.0, 0.0, 0.0};
    const lf::FrenetState target{9.0, 0.0, 0.0, 0.0};
    const auto candidate = lf::generate_candidate(initial, target, 0.5);
    EXPECT_TRUE(candidate.s_points.empty());
}

TEST(HardConstraint, DetectsBlockingObstacle)
{
    const auto grid = make_s_grid(0.0, 15.0, 0.5);
    const std::vector<double> centerline(grid.size(), 0.0);
    const std::vector<double> shifted(grid.size(), 1.0);
    EXPECT_FALSE(lf::is_collision_free(grid, centerline, 7.25, 0.0, 0.8, 0.5));
    EXPECT_TRUE(lf::is_collision_free(grid, shifted, 7.25, 0.0, 0.8, 0.5));
    EXPECT_FALSE(lf::is_collision_free(grid, shifted, 7.25, 1.7, 0.8, 0.5));
    EXPECT_TRUE(lf::is_collision_free(grid, centerline, 20.0, 0.0, 0.8, 0.5));
}

TEST(HardConstraint, CurvatureLimit)
{
    EXPECT_TRUE(lf::satisfies_curvature_limit({0.1, -0.2, 0.15}, 0.25));
    EXPECT_FALSE(lf::satisfies_curvature_limit({0.1, -0.3}, 0.25));
    EXPECT_TRUE(lf::satisfies_curvature_limit({}, 0.25));
}

TEST(SoftConstraint, CurvatureCostIsWeightedMeanOfAbsolutes)
{
    EXPECT_NEAR(lf::curvature_cost({0.1, -0.2, 0.3}, 2000.0), 400.0, 1e-9);
    EXPECT_DOUBLE_EQ(lf::curvature_cost({}, 2000.0), 0.0);
}

TEST(SoftConstraint, LengthCostRewardsLongerPath)
{
    EXPECT_DOUBLE_EQ(lf::length_cost(15.0, 1.0), -15.0);
    EXPECT_DOUBLE_EQ(lf::length_cost(0.0, 1.0), 0.0);
}

TEST(SoftConstraint, LateralDeviationCostPenalizesFinalOffset)
{
    EXPECT_DOUBLE_EQ(lf::lateral_deviation_cost(-0.8, 1.0), 0.8);
    EXPECT_DOUBLE_EQ(lf::lateral_deviation_cost(0.0, 1.0), 0.0);
}

TEST(SoftConstraint, CandidateCostSumsAllTerms)
{
    const lf::CostWeights weights{2000.0, 1.0, 1.0};
    EXPECT_NEAR(
        lf::candidate_cost({0.1, -0.2, 0.3}, 15.0, -0.8, weights),
        400.0 - 15.0 + 0.8, 1e-9);
    EXPECT_DOUBLE_EQ(lf::candidate_cost({}, 0.0, 0.0, weights), 0.0);
}

TEST(SoftConstraint, PrefersLowerCurvatureCandidate)
{
    const lf::CostWeights weights{2000.0, 1.0, 1.0};
    const std::vector<double> gentle(10, 0.05);
    const std::vector<double> sharp(10, 0.2);
    EXPECT_LT(
        lf::candidate_cost(gentle, 15.0, 0.0, weights),
        lf::candidate_cost(sharp, 15.0, 0.0, weights));
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
