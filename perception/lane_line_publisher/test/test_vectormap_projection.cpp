#include <gtest/gtest.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <camera_utility/ground_conversion.hpp>

#include "lane_line_publisher/vectormap_projection.hpp"

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

geometry_msgs::msg::Point make_point(const double x, const double y, const double z)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

visualization_msgs::msg::Marker make_line_strip_marker(
    const std::vector<geometry_msgs::msg::Point>& points,
    const float scale_x = 0.08F,
    const float alpha = 1.0F)
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = scale_x;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = alpha;
    marker.points = points;
    return marker;
}

}  // namespace

TEST(ProjectVectorMapMarkers, ProjectsLineStripPointsToPixels)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
        make_line_strip_marker({make_point(1.0, 0.0, 0.0), make_point(2.0, 0.0, 0.0)}));

    const auto projected = lane_line_publisher::project_vector_map_markers(
        marker_array, tf2::Transform::getIdentity(), simple_base_T_camera(), simple_intrinsics());

    ASSERT_EQ(projected.size(), 1U);
    ASSERT_EQ(projected[0].pixels.size(), 2U);

    cv::Point2f expected_first_pixel;
    ASSERT_TRUE(camera_utility::pointToPixel(
        tf2::Vector3(1.0, 0.0, 0.0), simple_intrinsics(), simple_base_T_camera(), expected_first_pixel));
    EXPECT_NEAR(projected[0].pixels[0].x, expected_first_pixel.x, 1.0e-3F);
    EXPECT_NEAR(projected[0].pixels[0].y, expected_first_pixel.y, 1.0e-3F);

    EXPECT_EQ(projected[0].color_bgr, cv::Scalar(0.0, 0.0, 255.0));
}

TEST(ProjectVectorMapMarkers, IgnoresDeleteAllAndNonLineStripMarkers)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    marker_array.markers.push_back(
        make_line_strip_marker({make_point(1.0, 0.0, 0.0), make_point(2.0, 0.0, 0.0)}));

    const auto projected = lane_line_publisher::project_vector_map_markers(
        marker_array, tf2::Transform::getIdentity(), simple_base_T_camera(), simple_intrinsics());

    EXPECT_EQ(projected.size(), 1U);
}

TEST(ProjectVectorMapMarkers, SplitsRunAtPointsBehindCamera)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(make_line_strip_marker({
        make_point(1.0, 0.0, 0.0),
        make_point(2.0, 0.0, 0.0),
        make_point(-5.0, 0.0, 1.0),
        make_point(3.0, 0.0, 0.0),
    }));

    cv::Point2f behind_pixel;
    ASSERT_FALSE(camera_utility::pointToPixel(
        tf2::Vector3(-5.0, 0.0, 1.0), simple_intrinsics(), simple_base_T_camera(), behind_pixel));

    const auto projected = lane_line_publisher::project_vector_map_markers(
        marker_array, tf2::Transform::getIdentity(), simple_base_T_camera(), simple_intrinsics());

    ASSERT_EQ(projected.size(), 1U);
    EXPECT_EQ(projected[0].pixels.size(), 2U);
}

TEST(ProjectVectorMapMarkers, ExcludesVirtualLineMarkers)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(make_line_strip_marker(
        {make_point(1.0, 0.0, 0.0), make_point(2.0, 0.0, 0.0)}, 0.04F, 0.45F));
    marker_array.markers.push_back(
        make_line_strip_marker({make_point(1.0, 0.0, 0.0), make_point(2.0, 0.0, 0.0)}));

    const auto projected = lane_line_publisher::project_vector_map_markers(
        marker_array, tf2::Transform::getIdentity(), simple_base_T_camera(), simple_intrinsics());

    EXPECT_EQ(projected.size(), 1U);
}

TEST(ProjectVectorMapMarkers, DropsSinglePointRuns)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
        make_line_strip_marker({make_point(1.0, 0.0, 0.0), make_point(-5.0, 0.0, 1.0)}));

    const auto projected = lane_line_publisher::project_vector_map_markers(
        marker_array, tf2::Transform::getIdentity(), simple_base_T_camera(), simple_intrinsics());

    EXPECT_TRUE(projected.empty());
}

TEST(DrawProjectedLineStrings, DrawsOntoImageWithoutThrowing)
{
    cv::Mat image = cv::Mat::zeros(360, 640, CV_8UC3);
    std::vector<lane_line_publisher::ProjectedLineString> line_strings;
    line_strings.push_back(lane_line_publisher::ProjectedLineString{
        {cv::Point2f(10.0F, 10.0F), cv::Point2f(100.0F, 100.0F)},
        cv::Scalar(0.0, 0.0, 255.0),
        2.0F});

    EXPECT_NO_THROW(lane_line_publisher::draw_projected_line_strings(image, line_strings));

    const auto pixel = image.at<cv::Vec3b>(10, 10);
    EXPECT_EQ(pixel, (cv::Vec3b{0, 0, 255}));
}
