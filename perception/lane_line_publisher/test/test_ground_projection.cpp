#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

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

TEST(BuildGroundProjectionLUT, ContainsProjectedPixelWithinDistanceLimits)
{
    const auto lut = lane_line_publisher::build_ground_projection_lut(
        simple_intrinsics(), simple_base_T_camera(), 0.0, 0.5, 10.0, 40);

    ASSERT_FALSE(lut.empty());

    bool found_center_pixel = false;
    for (const auto& entry : lut.entries) {
        EXPECT_EQ(entry.row % 40, 0);
        EXPECT_EQ(entry.col % 40, 0);
        EXPECT_LT(entry.row, 360);
        EXPECT_LT(entry.col, 640);

        const double distance = std::sqrt(
            static_cast<double>(entry.x_base) * entry.x_base +
            static_cast<double>(entry.y_base) * entry.y_base + 1.0);
        EXPECT_GE(distance, 0.5);
        EXPECT_LE(distance, 10.0);

        if (entry.row == 280 && entry.col == 320) {
            found_center_pixel = true;
            EXPECT_NEAR(entry.x_base, 1.0F, 1.0e-6F);
            EXPECT_NEAR(entry.y_base, 0.0F, 1.0e-6F);
        }
    }
    EXPECT_TRUE(found_center_pixel);
}

TEST(BuildGroundProjectionLUT, InvalidPixelStepThrows)
{
    EXPECT_THROW(
        lane_line_publisher::build_ground_projection_lut(
            simple_intrinsics(), simple_base_T_camera(), 0.0, 0.5, 10.0, 0),
        std::invalid_argument);
}
