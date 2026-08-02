#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "camera_utility/camera_intrinsics.hpp"
#include "camera_utility/camera_parameters.hpp"
#include "camera_utility/ground_conversion.hpp"

namespace
{

constexpr double kTolerance = 1.0e-9;

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

TEST(GetCameraIntrinsics, Nhd)
{
    const auto intrinsics = camera_utility::getCameraIntrinsics("nHD");
    EXPECT_EQ(intrinsics.width, 640);
    EXPECT_EQ(intrinsics.height, 360);
    EXPECT_DOUBLE_EQ(intrinsics.fx, 254.391622);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 254.391622);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 330.013020833);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 181.149637858);
}

TEST(GetCameraIntrinsics, Svga)
{
    const auto intrinsics = camera_utility::getCameraIntrinsics("SVGA");
    EXPECT_EQ(intrinsics.width, 960);
    EXPECT_EQ(intrinsics.height, 600);
    EXPECT_DOUBLE_EQ(intrinsics.fx, 377.3742370605469);
    EXPECT_DOUBLE_EQ(intrinsics.fy, 377.3742370605469);
    EXPECT_DOUBLE_EQ(intrinsics.cx, 495.00592041015625);
    EXPECT_DOUBLE_EQ(intrinsics.cy, 301.7193908691406);
}

TEST(GetCameraIntrinsics, UnknownThrows)
{
    EXPECT_THROW(camera_utility::getCameraIntrinsics("HD1080"), std::invalid_argument);
}

TEST(MakeTf2Transform, RotatesOpticalAxesWithDegreeInput)
{
    const auto base_T_camera = simple_base_T_camera();

    const auto forward = tf2::quatRotate(base_T_camera.getRotation(), tf2::Vector3(0.0, 0.0, 1.0));
    EXPECT_NEAR(forward.x(), 1.0, kTolerance);
    EXPECT_NEAR(forward.y(), 0.0, kTolerance);
    EXPECT_NEAR(forward.z(), 0.0, kTolerance);

    const auto right = tf2::quatRotate(base_T_camera.getRotation(), tf2::Vector3(1.0, 0.0, 0.0));
    EXPECT_NEAR(right.x(), 0.0, kTolerance);
    EXPECT_NEAR(right.y(), -1.0, kTolerance);
    EXPECT_NEAR(right.z(), 0.0, kTolerance);

    const auto down = tf2::quatRotate(base_T_camera.getRotation(), tf2::Vector3(0.0, 1.0, 0.0));
    EXPECT_NEAR(down.x(), 0.0, kTolerance);
    EXPECT_NEAR(down.y(), 0.0, kTolerance);
    EXPECT_NEAR(down.z(), -1.0, kTolerance);

    EXPECT_NEAR(base_T_camera.getOrigin().z(), 1.0, kTolerance);
}

TEST(PixelToPoint, ProjectsPixelOntoGround)
{
    tf2::Vector3 base_point;
    ASSERT_TRUE(camera_utility::pixelToPoint(
        cv::Point2f(320.0F, 280.0F), simple_intrinsics(), simple_base_T_camera(), base_point));
    EXPECT_NEAR(base_point.x(), 1.0, kTolerance);
    EXPECT_NEAR(base_point.y(), 0.0, kTolerance);
    EXPECT_NEAR(base_point.z(), 0.0, kTolerance);

    ASSERT_TRUE(camera_utility::pixelToPoint(
        cv::Point2f(420.0F, 280.0F), simple_intrinsics(), simple_base_T_camera(), base_point));
    EXPECT_NEAR(base_point.x(), 1.0, kTolerance);
    EXPECT_NEAR(base_point.y(), -1.0, kTolerance);
    EXPECT_NEAR(base_point.z(), 0.0, kTolerance);
}

TEST(PixelToPoint, RespectsGroundHeight)
{
    tf2::Vector3 base_point;
    ASSERT_TRUE(camera_utility::pixelToPoint(
        cv::Point2f(320.0F, 280.0F), simple_intrinsics(), simple_base_T_camera(), base_point, 0.5));
    EXPECT_NEAR(base_point.x(), 0.5, kTolerance);
    EXPECT_NEAR(base_point.y(), 0.0, kTolerance);
    EXPECT_NEAR(base_point.z(), 0.5, kTolerance);
}

TEST(PixelToPoint, RejectsPixelAboveHorizon)
{
    tf2::Vector3 base_point;
    EXPECT_FALSE(camera_utility::pixelToPoint(
        cv::Point2f(320.0F, 80.0F), simple_intrinsics(), simple_base_T_camera(), base_point));
}

TEST(PixelsToPoints, SkipsInvalidPixels)
{
    const std::vector<cv::Point2f> pixels{{320.0F, 280.0F}, {320.0F, 80.0F}};
    const auto base_points =
        camera_utility::pixelsToPoints(pixels, simple_intrinsics(), simple_base_T_camera());
    ASSERT_EQ(base_points.size(), 1U);
    EXPECT_NEAR(base_points.front().x(), 1.0, kTolerance);
    EXPECT_NEAR(base_points.front().y(), 0.0, kTolerance);
    EXPECT_NEAR(base_points.front().z(), 0.0, kTolerance);
}

TEST(PointToPixel, ProjectsGroundPointOntoImage)
{
    cv::Point2f pixel;
    ASSERT_TRUE(camera_utility::pointToPixel(
        tf2::Vector3(1.0, 0.0, 0.0), simple_intrinsics(), simple_base_T_camera(), pixel));
    EXPECT_NEAR(pixel.x, 320.0F, 1.0e-3F);
    EXPECT_NEAR(pixel.y, 280.0F, 1.0e-3F);

    ASSERT_TRUE(camera_utility::pointToPixel(
        tf2::Vector3(1.0, -1.0, 0.0), simple_intrinsics(), simple_base_T_camera(), pixel));
    EXPECT_NEAR(pixel.x, 420.0F, 1.0e-3F);
    EXPECT_NEAR(pixel.y, 280.0F, 1.0e-3F);
}

TEST(PointToPixel, RejectsPointBehindCamera)
{
    cv::Point2f pixel;
    EXPECT_FALSE(camera_utility::pointToPixel(
        tf2::Vector3(-2.0, 0.0, 0.0), simple_intrinsics(), simple_base_T_camera(), pixel));
}

TEST(GroundConversion, RoundTripWithVehicleParameters)
{
    const auto intrinsics = camera_utility::getCameraIntrinsics("nHD");
    const auto base_T_camera = camera_utility::makeTf2Transform(
        tf2::Vector3(0.055, 0.0, 0.54), -88.2, -0.2, -90.2);

    const cv::Point2f pixel(400.0F, 300.0F);
    tf2::Vector3 base_point;
    ASSERT_TRUE(camera_utility::pixelToPoint(pixel, intrinsics, base_T_camera, base_point));
    EXPECT_GT(base_point.x(), 0.0);
    EXPECT_NEAR(base_point.z(), 0.0, kTolerance);

    cv::Point2f reprojected_pixel;
    ASSERT_TRUE(camera_utility::pointToPixel(
        base_point, intrinsics, base_T_camera, reprojected_pixel));
    EXPECT_NEAR(reprojected_pixel.x, pixel.x, 1.0e-3F);
    EXPECT_NEAR(reprojected_pixel.y, pixel.y, 1.0e-3F);
}

TEST(CameraParameters, ReadsIntrinsicsAndExtrinsicsFromNode)
{
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
        {"camera.size", "SVGA"},
        {"camera.position.x", 0.5},
        {"camera.position.y", -0.1},
        {"camera.position.z", 1.2},
        {"camera.orientation.roll", -90.0},
        {"camera.orientation.pitch", 0.0},
        {"camera.orientation.yaw", -90.0},
    });
    rclcpp::Node node("camera_utility_test_node", options);

    const auto intrinsics = camera_utility::getCameraIntrinsics(node);
    EXPECT_EQ(intrinsics.width, 960);
    EXPECT_DOUBLE_EQ(intrinsics.fx, 377.3742370605469);

    const auto base_T_camera = camera_utility::getBaseTCamera(node);
    EXPECT_NEAR(base_T_camera.getOrigin().x(), 0.5, kTolerance);
    EXPECT_NEAR(base_T_camera.getOrigin().y(), -0.1, kTolerance);
    EXPECT_NEAR(base_T_camera.getOrigin().z(), 1.2, kTolerance);

    const auto forward = tf2::quatRotate(base_T_camera.getRotation(), tf2::Vector3(0.0, 0.0, 1.0));
    EXPECT_NEAR(forward.x(), 1.0, kTolerance);
    EXPECT_NEAR(forward.y(), 0.0, kTolerance);
    EXPECT_NEAR(forward.z(), 0.0, kTolerance);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
