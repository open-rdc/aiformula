#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>

#include <camera_utility/ground_conversion.hpp>

#include "lane_line_publisher/ground_projection.hpp"

namespace
{

camera_utility::CameraIntrinsics simple_intrinsics()
{
    return camera_utility::CameraIntrinsics{640, 360, 100.0, 100.0, 320.0, 180.0};
}

tf2::Transform simple_base_T_camera()
{
    return camera_utility::makeTf2Transform(
        tf2::Vector3(0.0, 0.0, 1.0), -90.0, 0.0, -90.0);
}

cv::Mat make_mask(int rows, int cols, const std::vector<cv::Point>& on_pixels)
{
    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (const auto& p : on_pixels) {
        mask.at<uint8_t>(p.y, p.x) = 255;
    }
    return mask;
}

}  // namespace

TEST(BuildGroundProjectionLUT, KnownCenterPixelMatchesPixelToPoint)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto look_up_table = lane_line_publisher::ground_projection_look_up_table(intrinsics, base_T_camera);

    tf2::Vector3 expected;
    ASSERT_TRUE(camera_utility::pixelToPoint(
        cv::Point2f(320.0F, 280.0F), intrinsics, base_T_camera, expected, 0.0));

    Eigen::Vector2d actual;
    ASSERT_TRUE(look_up_table.try_get(280, 320, actual));
    EXPECT_NEAR(actual.x(), expected.x(), 1e-9);
    EXPECT_NEAR(actual.y(), expected.y(), 1e-9);
}

TEST(BuildGroundProjectionLUT, OutOfBandLookupFails)
{
    const auto look_up_table = lane_line_publisher::ground_projection_look_up_table(
        simple_intrinsics(), simple_base_T_camera());

    Eigen::Vector2d unused;
    EXPECT_FALSE(look_up_table.try_get(look_up_table.row_begin - 1, 320, unused));
    EXPECT_FALSE(look_up_table.try_get(look_up_table.row_end, 320, unused));
    EXPECT_FALSE(look_up_table.try_get(look_up_table.row_begin, -1, unused));
    EXPECT_FALSE(look_up_table.try_get(look_up_table.row_begin, look_up_table.width, unused));
}

TEST(BuildGroundProjectionLUT, RowBandIsWithinDistanceLimitAndTightlyBounded)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto look_up_table = lane_line_publisher::ground_projection_look_up_table(intrinsics, base_T_camera);

    ASSERT_LT(look_up_table.row_begin, look_up_table.row_end);

    // 帯の中の有効セルは全て距離制限内でなければならない。
    for (int row = look_up_table.row_begin; row < look_up_table.row_end; ++row) {
        for (int col = 0; col < look_up_table.width; ++col) {
            Eigen::Vector2d point;
            if (look_up_table.try_get(row, col, point)) {
                EXPECT_LE(point.norm(), 10.0 + 1e-9);
            }
        }
    }

    // row_begin/row_end はタイトな境界でなければならない：帯のすぐ外側の行には、
    // LUTを使わず直接pixelToPointで調べても距離内の列が1つも無いはず。
    auto row_has_any_in_range_pixel = [&](int row) {
        for (int col = 0; col < intrinsics.width; ++col) {
            tf2::Vector3 point;
            if (!camera_utility::pixelToPoint(
                    cv::Point2f(static_cast<float>(col), static_cast<float>(row)),
                    intrinsics, base_T_camera, point, 0.0))
            {
                continue;
            }
            if ((point - base_T_camera.getOrigin()).length() <= 10.0) {
                return true;
            }
        }
        return false;
    };
    if (look_up_table.row_begin > 0) {
        EXPECT_FALSE(row_has_any_in_range_pixel(look_up_table.row_begin - 1));
    }
    if (look_up_table.row_end < look_up_table.height) {
        EXPECT_FALSE(row_has_any_in_range_pixel(look_up_table.row_end));
    }
}

TEST(LanePixelsToBasePoints, ProjectsAllValidMaskPixels)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto look_up_table = lane_line_publisher::ground_projection_look_up_table(intrinsics, base_T_camera);

    Eigen::Vector2d point_a, point_b;
    ASSERT_TRUE(look_up_table.try_get(look_up_table.row_begin, 320, point_a));
    ASSERT_TRUE(look_up_table.try_get(look_up_table.row_end - 1, 320, point_b));

    cv::Mat mask = cv::Mat::zeros(intrinsics.height, intrinsics.width, CV_8UC1);
    mask.at<uint8_t>(look_up_table.row_begin, 320) = 255;
    mask.at<uint8_t>(look_up_table.row_end - 1, 320) = 255;
    mask.at<uint8_t>(0, 0) = 255;

    const auto points = lane_line_publisher::lane_pixels_to_base_points(mask, look_up_table);

    ASSERT_EQ(points.size(), 2U);
    const bool contains_a = std::any_of(points.begin(), points.end(), [&](const Eigen::Vector2d& p) {
        return (p - point_a).norm() < 1e-9;
    });
    const bool contains_b = std::any_of(points.begin(), points.end(), [&](const Eigen::Vector2d& p) {
        return (p - point_b).norm() < 1e-9;
    });
    EXPECT_TRUE(contains_a);
    EXPECT_TRUE(contains_b);
}

TEST(LanePixelsToBasePoints, EmptyMaskReturnsNoPoints)
{
    const auto intrinsics = simple_intrinsics();
    const auto look_up_table = lane_line_publisher::ground_projection_look_up_table(
        intrinsics, simple_base_T_camera());
    const cv::Mat mask = cv::Mat::zeros(intrinsics.height, intrinsics.width, CV_8UC1);

    const auto points = lane_line_publisher::lane_pixels_to_base_points(mask, look_up_table);
    EXPECT_TRUE(points.empty());
}

TEST(VoxelDownsample, EmptyInput)
{
    EXPECT_TRUE(lane_line_publisher::voxel_downsample({}, 0.25).empty());
}

TEST(VoxelDownsample, PointsInSameVoxelAreAveraged)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.05, 0.05), Eigen::Vector2d(0.10, 0.05)};
    const auto result = lane_line_publisher::voxel_downsample(points, 0.25);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x(), 0.075, 1e-9);
    EXPECT_NEAR(result[0].y(), 0.05, 1e-9);
}

TEST(VoxelDownsample, PointsInDifferentVoxelsStaySeparate)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)};
    const auto result = lane_line_publisher::voxel_downsample(points, 0.25);
    EXPECT_EQ(result.size(), 2U);
}

TEST(VoxelDownsample, NonFinitePointsAreSkipped)
{
    const std::vector<Eigen::Vector2d> points{
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0)};
    const auto result = lane_line_publisher::voxel_downsample(points, 0.25);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
}

TEST(SplitLineComponents, DropsComponentsBelowMinPixelCount)
{
    const auto mask = make_mask(50, 50, {cv::Point(10, 10)});  // pixel_count == 1
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    EXPECT_TRUE(components.empty());
}

TEST(SplitLineComponents, KeepsComponentsAtOrAboveMinPixelCount)
{
    const auto mask = make_mask(
        50, 50, {cv::Point(10, 10), cv::Point(11, 10), cv::Point(12, 10),
                 cv::Point(13, 10), cv::Point(14, 10)});  // pixel_count == 5
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    ASSERT_EQ(components.size(), 1U);
    EXPECT_EQ(components[0].pixel_count, 5);
}

TEST(SplitLineComponents, SeparatesTwoDisconnectedComponents)
{
    const auto mask = make_mask(
        50, 50,
        {cv::Point(10, 10), cv::Point(11, 10), cv::Point(12, 10), cv::Point(13, 10), cv::Point(14, 10),
         cv::Point(30, 30), cv::Point(31, 30), cv::Point(32, 30), cv::Point(33, 30), cv::Point(34, 30)});
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    ASSERT_EQ(components.size(), 2U);
    EXPECT_EQ(components[0].pixel_count, 5);
    EXPECT_EQ(components[1].pixel_count, 5);
    EXPECT_EQ(cv::countNonZero(components[0].mask), 5);
    EXPECT_EQ(cv::countNonZero(components[1].mask), 5);
}
