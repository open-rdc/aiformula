#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

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

}  // namespace

TEST(BuildGroundProjectionLUT, KnownCenterPixelMatchesPixelToPoint)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        intrinsics, base_T_camera, 0.0, 10.0);

    tf2::Vector3 expected;
    ASSERT_TRUE(camera_utility::pixelToPoint(
        cv::Point2f(320.0F, 280.0F), intrinsics, base_T_camera, expected, 0.0));

    Eigen::Vector2d actual;
    ASSERT_TRUE(lut.try_get(280, 320, actual));
    EXPECT_NEAR(actual.x(), expected.x(), 1e-9);
    EXPECT_NEAR(actual.y(), expected.y(), 1e-9);
}

TEST(BuildGroundProjectionLUT, OutOfBandLookupFails)
{
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        simple_intrinsics(), simple_base_T_camera(), 0.0, 10.0);

    Eigen::Vector2d unused;
    EXPECT_FALSE(lut.try_get(lut.row_begin - 1, 320, unused));
    EXPECT_FALSE(lut.try_get(lut.row_end, 320, unused));
    EXPECT_FALSE(lut.try_get(lut.row_begin, -1, unused));
    EXPECT_FALSE(lut.try_get(lut.row_begin, lut.width, unused));
}

TEST(BuildGroundProjectionLUT, RowBandIsWithinDistanceLimitAndTightlyBounded)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        intrinsics, base_T_camera, 0.0, 10.0);

    ASSERT_LT(lut.row_begin, lut.row_end);

    // 帯の中の有効セルは全て距離制限内でなければならない。
    for (int row = lut.row_begin; row < lut.row_end; ++row) {
        for (int col = 0; col < lut.width; ++col) {
            Eigen::Vector2d point;
            if (lut.try_get(row, col, point)) {
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
    if (lut.row_begin > 0) {
        EXPECT_FALSE(row_has_any_in_range_pixel(lut.row_begin - 1));
    }
    if (lut.row_end < lut.height) {
        EXPECT_FALSE(row_has_any_in_range_pixel(lut.row_end));
    }
}

TEST(BuildGroundProjectionLUT, NonPositiveMaxDistanceThrows)
{
    EXPECT_THROW(
        lane_line_publisher::build_ground_projection_lut(
            simple_intrinsics(), simple_base_T_camera(), 0.0, 0.0),
        std::invalid_argument);
}

TEST(LanePixelsToBasePoints, PrioritizesNearestPointsWhenCappingAtMaxPoints)
{
    const auto intrinsics = simple_intrinsics();
    const auto base_T_camera = simple_base_T_camera();
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        intrinsics, base_T_camera, 0.0, 10.0);

    Eigen::Vector2d point_a, point_b;
    ASSERT_TRUE(lut.try_get(lut.row_begin, 320, point_a));
    ASSERT_TRUE(lut.try_get(lut.row_end - 1, 320, point_b));
    ASSERT_NE(point_a.norm(), point_b.norm());
    const int near_row = (point_a.norm() < point_b.norm()) ? lut.row_begin : (lut.row_end - 1);
    const int far_row = (point_a.norm() < point_b.norm()) ? (lut.row_end - 1) : lut.row_begin;

    cv::Mat mask = cv::Mat::zeros(intrinsics.height, intrinsics.width, CV_8UC1);
    mask.at<uint8_t>(near_row, 320) = 255;
    mask.at<uint8_t>(far_row, 320) = 255;

    const auto points = lane_line_publisher::lane_pixels_to_base_points(mask, lut, 1U);

    ASSERT_EQ(points.size(), 1U);
    Eigen::Vector2d expected_near_point;
    ASSERT_TRUE(lut.try_get(near_row, 320, expected_near_point));
    EXPECT_NEAR(points[0].x(), expected_near_point.x(), 1e-9);
    EXPECT_NEAR(points[0].y(), expected_near_point.y(), 1e-9);
}

TEST(LanePixelsToBasePoints, EmptyMaskReturnsNoPoints)
{
    const auto intrinsics = simple_intrinsics();
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        intrinsics, simple_base_T_camera(), 0.0, 10.0);
    const cv::Mat mask = cv::Mat::zeros(intrinsics.height, intrinsics.width, CV_8UC1);

    const auto points = lane_line_publisher::lane_pixels_to_base_points(mask, lut, 100U);
    EXPECT_TRUE(points.empty());
}
