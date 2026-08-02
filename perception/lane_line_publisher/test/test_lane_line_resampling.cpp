#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include <Eigen/Core>

#include "lane_line_publisher/lane_line_resampling.hpp"

namespace lane_line_publisher
{
namespace
{

double max_consecutive_distance(const std::vector<Eigen::Vector2d>& points)
{
    double max_distance = 0.0;
    for (std::size_t i = 1U; i < points.size(); ++i) {
        max_distance = std::max(max_distance, (points[i] - points[i - 1U]).norm());
    }
    return max_distance;
}

bool contains_point(const std::vector<Eigen::Vector2d>& points, const Eigen::Vector2d& target)
{
    return std::any_of(points.begin(), points.end(), [&](const Eigen::Vector2d& p) {
        return (p - target).norm() < 1e-9;
    });
}

}  // namespace

// --- cluster_points_into_lines ---

TEST(ClusterPointsIntoLines, EmptyInput)
{
    EXPECT_TRUE(cluster_points_into_lines({}, 0.5).empty());
}

TEST(ClusterPointsIntoLines, SinglePointFormsOneCluster)
{
    const auto clusters = cluster_points_into_lines({Eigen::Vector2d(1.0, 2.0)}, 0.5);
    ASSERT_EQ(clusters.size(), 1U);
    ASSERT_EQ(clusters[0].size(), 1U);
    EXPECT_NEAR(clusters[0][0].x(), 1.0, 1e-9);
    EXPECT_NEAR(clusters[0][0].y(), 2.0, 1e-9);
}

TEST(ClusterPointsIntoLines, TwoClosePointsMergeIntoOneCluster)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.3, 0.0)};
    const auto clusters = cluster_points_into_lines(points, 0.5);
    ASSERT_EQ(clusters.size(), 1U);
    EXPECT_EQ(clusters[0].size(), 2U);
}

TEST(ClusterPointsIntoLines, TwoFarPointsStaySeparate)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 0.0)};
    const auto clusters = cluster_points_into_lines(points, 0.5);
    ASSERT_EQ(clusters.size(), 2U);
    EXPECT_EQ(clusters[0].size(), 1U);
    EXPECT_EQ(clusters[1].size(), 1U);
}

TEST(ClusterPointsIntoLines, ShuffledLinePointsFormSingleChain)
{
    // A straight line with even spacing, fed in shuffled order. Growth must
    // proceed bidirectionally from whichever point happens to be visited first.
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(4.0, 0.0),
        Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(2.0, 0.0)};
    const auto clusters = cluster_points_into_lines(points, 1.5);
    ASSERT_EQ(clusters.size(), 1U);
    EXPECT_EQ(clusters[0].size(), 5U);
    EXPECT_LE(max_consecutive_distance(clusters[0]), 1.5 + 1e-9);
    for (const auto& p : points) {
        EXPECT_TRUE(contains_point(clusters[0], p));
    }
}

TEST(ClusterPointsIntoLines, TwoDistinctLinesStaySeparate)
{
    const std::vector<Eigen::Vector2d> line_a{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.2, 0.0), Eigen::Vector2d(0.4, 0.0),
        Eigen::Vector2d(0.6, 0.0)};
    const std::vector<Eigen::Vector2d> line_b{
        Eigen::Vector2d(3.0, 5.0), Eigen::Vector2d(3.2, 5.0), Eigen::Vector2d(3.4, 5.0),
        Eigen::Vector2d(3.6, 5.0)};

    std::vector<Eigen::Vector2d> shuffled{
        line_b[2], line_a[0], line_b[0], line_a[3], line_b[3], line_a[1], line_b[1], line_a[2]};

    const auto clusters = cluster_points_into_lines(shuffled, 0.5);
    ASSERT_EQ(clusters.size(), 2U);
    EXPECT_EQ(clusters[0].size() + clusters[1].size(), 8U);

    for (const auto& p : line_a) {
        const bool in_first = contains_point(clusters[0], p);
        const bool in_second = contains_point(clusters[1], p);
        EXPECT_TRUE(in_first != in_second);
    }
}

// --- resample_polyline_by_distance ---

TEST(ResamplePolylineByDistance, EmptyInput)
{
    EXPECT_TRUE(resample_polyline_by_distance({}, 1.0).empty());
}

TEST(ResamplePolylineByDistance, SinglePointReturnsSamePoint)
{
    const std::vector<Eigen::Vector2d> points{Eigen::Vector2d(1.0, 1.0)};
    const auto result = resample_polyline_by_distance(points, 1.0);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x(), 1.0, 1e-9);
}

TEST(ResamplePolylineByDistance, ExactMultipleOfIntervalIsEvenlySpaced)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(9.0, 0.0)};
    const auto result = resample_polyline_by_distance(points, 3.0);
    ASSERT_EQ(result.size(), 4U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
    EXPECT_NEAR(result[1].x(), 3.0, 1e-9);
    EXPECT_NEAR(result[2].x(), 6.0, 1e-9);
    EXPECT_NEAR(result[3].x(), 9.0, 1e-9);
}

TEST(ResamplePolylineByDistance, NonMultipleIncludesFinalEndpoint)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(10.0, 0.0)};
    const auto result = resample_polyline_by_distance(points, 3.0);
    ASSERT_EQ(result.size(), 5U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
    EXPECT_NEAR(result[1].x(), 3.0, 1e-9);
    EXPECT_NEAR(result[2].x(), 6.0, 1e-9);
    EXPECT_NEAR(result[3].x(), 9.0, 1e-9);
    EXPECT_NEAR(result[4].x(), 10.0, 1e-9);
}

TEST(ResamplePolylineByDistance, MultiSegmentCarriesRemainderAcrossCorner)
{
    // (0,0) -> (5,0) -> (5,5), interval 3: samples at cumulative arc length
    // 3, 6, 9, plus the forced final endpoint at arc length 10.
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(5.0, 5.0)};
    const auto result = resample_polyline_by_distance(points, 3.0);
    ASSERT_EQ(result.size(), 5U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
    EXPECT_NEAR(result[0].y(), 0.0, 1e-9);
    EXPECT_NEAR(result[1].x(), 3.0, 1e-9);
    EXPECT_NEAR(result[1].y(), 0.0, 1e-9);
    EXPECT_NEAR(result[2].x(), 5.0, 1e-9);
    EXPECT_NEAR(result[2].y(), 1.0, 1e-9);
    EXPECT_NEAR(result[3].x(), 5.0, 1e-9);
    EXPECT_NEAR(result[3].y(), 4.0, 1e-9);
    EXPECT_NEAR(result[4].x(), 5.0, 1e-9);
    EXPECT_NEAR(result[4].y(), 5.0, 1e-9);
}

TEST(ResamplePolylineByDistance, NonPositiveIntervalReturnsInputUnchanged)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)};
    const auto result_zero = resample_polyline_by_distance(points, 0.0);
    ASSERT_EQ(result_zero.size(), points.size());
    for (std::size_t i = 0U; i < points.size(); ++i) {
        EXPECT_TRUE(result_zero[i].isApprox(points[i]));
    }

    const auto result_negative = resample_polyline_by_distance(points, -1.0);
    ASSERT_EQ(result_negative.size(), points.size());
    for (std::size_t i = 0U; i < points.size(); ++i) {
        EXPECT_TRUE(result_negative[i].isApprox(points[i]));
    }
}

TEST(ResamplePolylineByDistance, DegenerateDuplicatePointsAreSkipped)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(6.0, 0.0)};
    const auto result = resample_polyline_by_distance(points, 3.0);
    ASSERT_EQ(result.size(), 3U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
    EXPECT_NEAR(result[1].x(), 3.0, 1e-9);
    EXPECT_NEAR(result[2].x(), 6.0, 1e-9);
}

// --- resample_lane_points (integration) ---

TEST(ResampleLanePoints, TwoLinesAreResampledIndependently)
{
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i <= 9; ++i) {
        points.emplace_back(static_cast<double>(i) * 0.2, 0.0);
    }
    for (int i = 0; i <= 9; ++i) {
        points.emplace_back(3.0 + static_cast<double>(i) * 0.2, 5.0);
    }
    std::reverse(points.begin(), points.begin() + 10);

    const auto result = resample_lane_points(points, 0.5, 0.5);

    int near_line_a = 0;
    int near_line_b = 0;
    for (const auto& p : result) {
        if (std::abs(p.y() - 0.0) < 1e-6) {
            ++near_line_a;
        } else if (std::abs(p.y() - 5.0) < 1e-6) {
            ++near_line_b;
        }
    }
    EXPECT_EQ(near_line_a + near_line_b, static_cast<int>(result.size()));
    // line length 1.8m, interval 0.5m -> samples at 0,0.5,1.0,1.5,1.8 = 5 points each
    EXPECT_EQ(near_line_a, 5);
    EXPECT_EQ(near_line_b, 5);
}

TEST(ResampleLanePoints, EmptyInputReturnsEmpty)
{
    EXPECT_TRUE(resample_lane_points({}, 0.5, 0.25).empty());
}

// --- voxel_downsample ---

TEST(VoxelDownsample, EmptyInput)
{
    EXPECT_TRUE(voxel_downsample({}, 0.25).empty());
}

TEST(VoxelDownsample, PointsInSameVoxelAreAveraged)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.05, 0.05), Eigen::Vector2d(0.10, 0.05)};
    const auto result = voxel_downsample(points, 0.25);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x(), 0.075, 1e-9);
    EXPECT_NEAR(result[0].y(), 0.05, 1e-9);
}

TEST(VoxelDownsample, PointsInDifferentVoxelsStaySeparate)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)};
    const auto result = voxel_downsample(points, 0.25);
    EXPECT_EQ(result.size(), 2U);
}

TEST(VoxelDownsample, NonFinitePointsAreSkipped)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0)};
    const auto result = voxel_downsample(points, 0.25);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
}

// --- resample_lane_point_groups ---

TEST(ResampleLanePointGroups, DistinctGroupsAreNotBridgedEvenWhenClose)
{
    // group_bの先頭とgroup_aの末尾の間隔(0.05)はmax_link_distance_m(0.5)未満だが、
    // 別々の連結成分から来た点なので橋渡しされてはならない。
    const std::vector<Eigen::Vector2d> group_a{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.2, 0.0), Eigen::Vector2d(0.4, 0.0)};
    const std::vector<Eigen::Vector2d> group_b{
        Eigen::Vector2d(0.45, 0.0), Eigen::Vector2d(0.65, 0.0), Eigen::Vector2d(0.85, 0.0)};
    ASSERT_LT((group_b.front() - group_a.back()).norm(), 0.5);

    const auto result = resample_lane_point_groups({group_a, group_b}, 0.5, 1.0, 0.1);

    // 橋渡しされていれば0.0-0.85mの1本の線として区間1.0mでリサンプルされ、
    // 端点は0.0と0.85のみになる。橋渡しされていなければ、各グループが独立に
    // リサンプルされ0.4と0.45の両方が結果に残る。
    bool has_point_at_0_4 = false;
    bool has_point_at_0_45 = false;
    for (const auto& p : result) {
        if (std::abs(p.x() - 0.4) < 1e-6) has_point_at_0_4 = true;
        if (std::abs(p.x() - 0.45) < 1e-6) has_point_at_0_45 = true;
    }
    EXPECT_TRUE(has_point_at_0_4);
    EXPECT_TRUE(has_point_at_0_45);
}

TEST(ResampleLanePointGroups, EmptyGroupsReturnsEmpty)
{
    EXPECT_TRUE(resample_lane_point_groups({}, 0.5, 0.25, 0.25).empty());
}

}  // namespace lane_line_publisher
