#include "lane_line_publisher/ground_projection.hpp"

#include <stdexcept>

#include <camera_utility/ground_conversion.hpp>

namespace lane_line_publisher
{

GroundProjectionLUT build_ground_projection_lut(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    const double ground_z,
    const double min_ground_intersection_distance,
    const double max_ground_intersection_distance,
    const int pixel_step)
{
    if (pixel_step <= 0) {
        throw std::invalid_argument("pixel_step must be greater than 0");
    }
    if (intrinsics.width <= 0 || intrinsics.height <= 0) {
        throw std::invalid_argument("image dimensions must be greater than 0");
    }

    GroundProjectionLUT lut;
    lut.entries.reserve(
        static_cast<std::size_t>((intrinsics.width / pixel_step) + 1) *
        static_cast<std::size_t>((intrinsics.height / pixel_step) + 1));

    for (int row = 0; row < intrinsics.height; row += pixel_step) {
        for (int col = 0; col < intrinsics.width; col += pixel_step) {
            tf2::Vector3 base_point;
            if (!camera_utility::pixelToPoint(
                    cv::Point2f(static_cast<float>(col), static_cast<float>(row)),
                    intrinsics, base_T_camera, base_point, ground_z))
            {
                continue;
            }

            const double distance = (base_point - base_T_camera.getOrigin()).length();
            if (distance < min_ground_intersection_distance ||
                distance > max_ground_intersection_distance)
            {
                continue;
            }

            lut.entries.push_back(GroundProjectionEntry{
                row,
                col,
                static_cast<float>(base_point.x()),
                static_cast<float>(base_point.y())});
        }
    }

    return lut;
}

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& lut,
    const uint8_t mask_threshold,
    const std::size_t max_points)
{
    std::vector<Eigen::Vector2d> base_points;
    base_points.reserve(std::min(max_points, lut.entries.size()));
    for (const auto& entry : lut.entries) {
        if (skeleton_mask.at<uint8_t>(entry.row, entry.col) < mask_threshold) {
            continue;
        }
        base_points.emplace_back(
            static_cast<double>(entry.x_base),
            static_cast<double>(entry.y_base));
        if (base_points.size() >= max_points) {
            return base_points;
        }
    }
    return base_points;
}

}
