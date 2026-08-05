#include <gtest/gtest.h>

#include <unordered_map>
#include <vector>

#include "mission_planner/route_builder.hpp"

namespace mp = mission_planner;

TEST(MengerCurvature, IsZeroForColinearPoints)
{
    const mp::Point2D a{0.0, 0.0};
    const mp::Point2D b{1.0, 0.0};
    const mp::Point2D c{2.0, 0.0};
    EXPECT_NEAR(mp::menger_curvature(a, b, c), 0.0, 1e-9);
}

TEST(MengerCurvature, MatchesKnownRightAngleTurn)
{
    const mp::Point2D a{0.0, 0.0};
    const mp::Point2D b{1.0, 0.0};
    const mp::Point2D c{1.0, 1.0};
    EXPECT_NEAR(mp::menger_curvature(a, b, c), std::sqrt(2.0), 1e-6);
}

TEST(CatmullRomSmooth, ReturnsInputUnchangedForFewerThanTwoPoints)
{
    const std::vector<mp::Point2D> control_points{{1.0, 2.0}};
    const auto result = mp::catmull_rom_smooth(control_points, 10);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x, 1.0, 1e-9);
    EXPECT_NEAR(result[0].y, 2.0, 1e-9);
}

TEST(CatmullRomSmooth, PassesThroughAllControlPointsOnStraightLine)
{
    const std::vector<mp::Point2D> control_points{
        {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0}};
    const auto result = mp::catmull_rom_smooth(control_points, 4);
    ASSERT_FALSE(result.empty());
    for (const auto& point : result) {
        EXPECT_NEAR(point.y, 0.0, 1e-6);
    }
    EXPECT_NEAR(result.front().x, 0.0, 1e-6);
    EXPECT_NEAR(result.back().x, 6.0, 1e-6);
}

TEST(SelectStartLanelet, PicksNearestLaneletWithinYawThreshold)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {10.0, 0.0}};
    centerlines[2U] = {{0.0, 5.0}, {10.0, 5.0}};

    const uint64_t result = mp::select_start_lanelet(
        centerlines, mp::Point2D{5.0, 0.2}, 0.0, 0.7854, 5.0);
    EXPECT_EQ(result, 1U);
}

TEST(SelectStartLanelet, RejectsLaneletBeyondYawThreshold)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {10.0, 0.0}};

    const uint64_t result = mp::select_start_lanelet(
        centerlines, mp::Point2D{5.0, 0.2}, M_PI_2, 0.1, 5.0);
    EXPECT_EQ(result, 0U);
}

TEST(SelectStartLanelet, RejectsLaneletBeyondMaxDistance)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {10.0, 0.0}};

    const uint64_t result = mp::select_start_lanelet(
        centerlines, mp::Point2D{5.0, 50.0}, 0.0, 0.7854, 5.0);
    EXPECT_EQ(result, 0U);
}

namespace
{
constexpr uint8_t kTurnStraight = 1U;
constexpr uint8_t kTurnLeft = 2U;
constexpr uint8_t kTurnRight = 3U;
}

TEST(SelectNextLaneletId, PicksRequestedTurnWhenAvailable)
{
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {
        mp::RouteEdge{10U, kTurnStraight, 1.0},
        mp::RouteEdge{20U, kTurnLeft, 1.0},
    };
    bool used_fallback = false;
    const uint64_t result = mp::select_next_lanelet_id(edges, 1U, kTurnLeft, {kTurnStraight}, used_fallback);
    EXPECT_EQ(result, 20U);
    EXPECT_FALSE(used_fallback);
}

TEST(SelectNextLaneletId, FallsBackToOrderWhenRequestedTurnUnavailable)
{
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {mp::RouteEdge{10U, kTurnStraight, 1.0}};
    bool used_fallback = false;
    const uint64_t result = mp::select_next_lanelet_id(
        edges, 1U, kTurnRight, {kTurnStraight, kTurnLeft}, used_fallback);
    EXPECT_EQ(result, 10U);
    EXPECT_TRUE(used_fallback);
}

TEST(SelectNextLaneletId, ReturnsZeroWhenNoEdgeAndNoFallbackMatch)
{
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {mp::RouteEdge{10U, kTurnLeft, 1.0}};
    bool used_fallback = false;
    const uint64_t result = mp::select_next_lanelet_id(edges, 1U, kTurnRight, {kTurnRight}, used_fallback);
    EXPECT_EQ(result, 0U);
}

TEST(SelectNextLaneletId, ReturnsZeroWhenFromLaneletHasNoEdges)
{
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    bool used_fallback = false;
    const uint64_t result = mp::select_next_lanelet_id(edges, 99U, kTurnStraight, {}, used_fallback);
    EXPECT_EQ(result, 0U);
}

TEST(FindRouteLaneletIds, WalksGraphUpToLookaheadCount)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {1.0, 0.0}};
    centerlines[2U] = {{1.0, 0.0}, {2.0, 0.0}};
    centerlines[3U] = {{2.0, 0.0}, {3.0, 0.0}};
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {mp::RouteEdge{2U, kTurnStraight, 1.0}};
    edges[2U] = {mp::RouteEdge{3U, kTurnStraight, 1.0}};

    std::size_t fallback_count = 0U;
    const auto result = mp::find_route_lanelet_ids(
        centerlines, edges, {kTurnStraight}, 1U, kTurnStraight, 3, fallback_count);
    ASSERT_EQ(result.size(), 3U);
    EXPECT_EQ(result[0], 1U);
    EXPECT_EQ(result[1], 2U);
    EXPECT_EQ(result[2], 3U);
    EXPECT_EQ(fallback_count, 0U);
}

TEST(FindRouteLaneletIds, StopsWhenNoOutgoingEdge)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {1.0, 0.0}};
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;

    std::size_t fallback_count = 0U;
    const auto result = mp::find_route_lanelet_ids(
        centerlines, edges, {kTurnStraight}, 1U, kTurnStraight, 5, fallback_count);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_EQ(result[0], 1U);
}

TEST(FindRouteLaneletIds, StopsOnCycleBackToVisitedLanelet)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {1.0, 0.0}};
    centerlines[2U] = {{1.0, 0.0}, {2.0, 0.0}};
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {mp::RouteEdge{2U, kTurnStraight, 1.0}};
    edges[2U] = {mp::RouteEdge{1U, kTurnStraight, 1.0}};

    std::size_t fallback_count = 0U;
    const auto result = mp::find_route_lanelet_ids(
        centerlines, edges, {kTurnStraight}, 1U, kTurnStraight, 10, fallback_count);
    ASSERT_EQ(result.size(), 2U);
    EXPECT_EQ(result[0], 1U);
    EXPECT_EQ(result[1], 2U);
}

TEST(FindRouteLaneletIds, CountsFallbackUsage)
{
    std::unordered_map<uint64_t, std::vector<mp::Point2D>> centerlines;
    centerlines[1U] = {{0.0, 0.0}, {1.0, 0.0}};
    centerlines[2U] = {{1.0, 0.0}, {2.0, 0.0}};
    std::unordered_map<uint64_t, std::vector<mp::RouteEdge>> edges;
    edges[1U] = {mp::RouteEdge{2U, kTurnLeft, 1.0}};

    std::size_t fallback_count = 0U;
    const auto result = mp::find_route_lanelet_ids(
        centerlines, edges, {kTurnLeft}, 1U, kTurnStraight, 2, fallback_count);
    ASSERT_EQ(result.size(), 2U);
    EXPECT_EQ(fallback_count, 1U);
}
