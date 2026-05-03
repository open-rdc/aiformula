#include "localization/lane_pixel_to_point.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace localization
{

Eigen::Matrix3d rotation_matrix_from_rpy(const double roll, const double pitch, const double yaw)
{
    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

GroundProjectionLUT build_ground_projection_lut(
    const CameraModel& camera_model,
    const int pixel_step,
    const int image_width,
    const int image_height)
{
    if (pixel_step <= 0) {
        throw std::invalid_argument("pixel_step must be greater than 0");
    }
    if (image_width <= 0 || image_height <= 0) {
        throw std::invalid_argument("image dimensions must be greater than 0");
    }

    GroundProjectionLUT lut;
    lut.image_width = image_width;
    lut.image_height = image_height;
    lut.entries.reserve(
        static_cast<std::size_t>((image_width / pixel_step) + 1) *
        static_cast<std::size_t>((image_height / pixel_step) + 1));

    for (int row = 0; row < image_height; row += pixel_step) {
        for (int col = 0; col < image_width; col += pixel_step) {
            const Eigen::Vector3d camera_ray(
                (static_cast<double>(col) - camera_model.cx) / camera_model.fx,
                (static_cast<double>(row) - camera_model.cy) / camera_model.fy,
                1.0);
            const Eigen::Vector3d base_ray = camera_model.camera_to_base_rotation * camera_ray;
            if (std::abs(base_ray.z()) < 1.0e-9) {
                continue;
            }

            const double scale =
                (camera_model.ground_plane_z_base - camera_model.camera_to_base_translation.z()) /
                base_ray.z();
            if (scale <= 0.0 || !std::isfinite(scale)) {
                continue;
            }

            const Eigen::Vector3d base_point =
                camera_model.camera_to_base_translation + scale * base_ray;
            const double distance =
                (base_point - camera_model.camera_to_base_translation).norm();
            if (distance < camera_model.min_ground_intersection_distance ||
                distance > camera_model.max_ground_intersection_distance)
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
    if (skeleton_mask.empty()) {
        throw std::invalid_argument("skeleton_mask must not be empty");
    }
    if (skeleton_mask.type() != CV_8UC1) {
        throw std::invalid_argument("skeleton_mask must be CV_8UC1");
    }
    if (skeleton_mask.cols != lut.image_width || skeleton_mask.rows != lut.image_height) {
        throw std::invalid_argument("skeleton_mask size does not match ground projection LUT");
    }
    if (lut.empty()) {
        throw std::invalid_argument("ground projection LUT must not be empty");
    }
    if (max_points == 0U) {
        throw std::invalid_argument("max_points must be greater than 0");
    }

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

}  // namespace localization
