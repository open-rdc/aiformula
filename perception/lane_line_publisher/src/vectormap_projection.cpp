#include "lane_line_publisher/vectormap_projection.hpp"

#include <algorithm>
#include <cmath>

#include <camera_utility/ground_conversion.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace lane_line_publisher
{
namespace
{

cv::Scalar to_bgr_scalar(const std_msgs::msg::ColorRGBA& color)
{
    return cv::Scalar(
        static_cast<double>(color.b) * 255.0,
        static_cast<double>(color.g) * 255.0,
        static_cast<double>(color.r) * 255.0);
}

float to_thickness_px(const double scale_x_m)
{
    constexpr double reference_line_width_m = 0.04;
    return static_cast<float>(std::max(1.0, std::round(scale_x_m / reference_line_width_m)));
}

bool is_virtual_line_marker(const visualization_msgs::msg::Marker& marker)
{
    constexpr float solid_alpha = 1.0F;
    return marker.color.a < solid_alpha;
}

}  // namespace

std::vector<ProjectedLineString> project_vector_map_markers(
    const visualization_msgs::msg::MarkerArray& marker_array,
    const tf2::Transform& base_T_map,
    const tf2::Transform& base_T_camera,
    const camera_utility::CameraIntrinsics& intrinsics)
{
    std::vector<ProjectedLineString> projected_line_strings;

    for (const auto& marker : marker_array.markers) {
        if (marker.action != visualization_msgs::msg::Marker::ADD ||
            marker.type != visualization_msgs::msg::Marker::LINE_STRIP)
        {
            continue;
        }
        if (is_virtual_line_marker(marker)) {
            continue;
        }

        const cv::Scalar color_bgr = to_bgr_scalar(marker.color);
        const float thickness_px = to_thickness_px(marker.scale.x);

        std::vector<cv::Point2f> pixel_run;
        for (const auto& map_point : marker.points) {
            const tf2::Vector3 base_point =
                base_T_map * tf2::Vector3(map_point.x, map_point.y, map_point.z);

            cv::Point2f pixel;
            if (camera_utility::pointToPixel(base_point, intrinsics, base_T_camera, pixel)) {
                pixel_run.push_back(pixel);
                continue;
            }

            if (pixel_run.size() >= 2U) {
                projected_line_strings.push_back(ProjectedLineString{pixel_run, color_bgr, thickness_px});
            }
            pixel_run.clear();
        }

        if (pixel_run.size() >= 2U) {
            projected_line_strings.push_back(ProjectedLineString{pixel_run, color_bgr, thickness_px});
        }
    }

    return projected_line_strings;
}

void draw_projected_line_strings(
    cv::Mat& image,
    const std::vector<ProjectedLineString>& line_strings)
{
    for (const auto& line_string : line_strings) {
        std::vector<cv::Point> points;
        points.reserve(line_string.pixels.size());
        for (const auto& pixel : line_string.pixels) {
            points.emplace_back(cvRound(pixel.x), cvRound(pixel.y));
        }
        cv::polylines(
            image, points, false, line_string.color_bgr,
            static_cast<int>(line_string.thickness_px), cv::LINE_AA);
    }
}

}
