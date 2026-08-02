#include <gtest/gtest.h>

#include <vector>

#include "mission_planner/route_builder.hpp"

namespace mp = mission_planner;

namespace
{
constexpr double kDefaultMinGapM = 0.5;

void expect_point_near(const mp::Point2D& actual, const mp::Point2D& expected)
{
    EXPECT_NEAR(actual.x, expected.x, 1e-9);
    EXPECT_NEAR(actual.y, expected.y, 1e-9);
}
}

TEST(RoutePointDedup, KeepsAllPointsWhenSpacedWellAboveMinGap)
{
    const std::vector<mp::Point2D> first_lanelet{
        {0.0, 0.0}, {3.0, 0.0}, {6.0, 0.0}};
    const std::vector<mp::Point2D> second_lanelet{
        {6.0, 0.0}, {9.0, 0.0}, {12.0, 0.0}};

    std::vector<mp::Point2D> route_points;
    mp::append_centerline_points(route_points, first_lanelet, false, kDefaultMinGapM);
    mp::append_centerline_points(route_points, second_lanelet, true, kDefaultMinGapM);

    ASSERT_EQ(route_points.size(), 5U);
    expect_point_near(route_points[0], {0.0, 0.0});
    expect_point_near(route_points[1], {3.0, 0.0});
    expect_point_near(route_points[2], {6.0, 0.0});
    expect_point_near(route_points[3], {9.0, 0.0});
    expect_point_near(route_points[4], {12.0, 0.0});
}

// 実車ログで確認された誤検出曲率(kappa=0.304, limit=0.268, s=44.80m)の回帰テスト
TEST(RoutePointDedup, MergesShortSegmentFromRealCourseData)
{
    const mp::Point2D node130{34.31270230522499, -35.63960241622622};
    const mp::Point2D node131{37.16063565509035, -36.580535549401674};
    const mp::Point2D node132{37.358395450274145, -36.64417576089107};
    const mp::Point2D node208{37.39465370547941, -36.64741167203639};
    const mp::Point2D node209{40.31342107770219, -37.33892790176782};

    const std::vector<mp::Point2D> lanelet_304_centerline{node130, node131, node132};
    const std::vector<mp::Point2D> lanelet_305_centerline{node208, node209};

    std::vector<mp::Point2D> route_points;
    mp::append_centerline_points(route_points, lanelet_304_centerline, false, kDefaultMinGapM);
    mp::append_centerline_points(route_points, lanelet_305_centerline, true, kDefaultMinGapM);

    ASSERT_EQ(route_points.size(), 3U);
    expect_point_near(route_points[0], node130);
    expect_point_near(route_points[1], node131);
    expect_point_near(route_points[2], node209);
}

TEST(RoutePointDedup, MergesPointExactlyAtMinGapThreshold)
{
    const std::vector<mp::Point2D> centerline{{0.0, 0.0}, {0.5, 0.0}, {5.0, 0.0}};

    std::vector<mp::Point2D> route_points;
    mp::append_centerline_points(route_points, centerline, false, kDefaultMinGapM);

    ASSERT_EQ(route_points.size(), 2U);
    expect_point_near(route_points[0], {0.0, 0.0});
    expect_point_near(route_points[1], {5.0, 0.0});
}

TEST(RoutePointDedup, KeepsFirstPointWhenNotConnectedToPrevious)
{
    const std::vector<mp::Point2D> centerline{{1.0, 1.0}, {4.0, 1.0}};

    std::vector<mp::Point2D> route_points;
    mp::append_centerline_points(route_points, centerline, false, kDefaultMinGapM);

    ASSERT_EQ(route_points.size(), 2U);
    expect_point_near(route_points[0], {1.0, 1.0});
}
