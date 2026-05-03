#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace localization
{

struct CameraModel
{
    double fx;
    double fy;
    double cx;
    double cy;
    double ground_plane_z_base;
    double min_ground_intersection_distance;
    double max_ground_intersection_distance;
    Eigen::Matrix3d camera_to_base_rotation;
    Eigen::Vector3d camera_to_base_translation;
};

struct GroundProjectionEntry
{
    int row;
    int col;
    float x_base;
    float y_base;
};

// Stores valid ground-projectable pixel locations sampled at pixel_step intervals.
struct GroundProjectionLUT
{
    int image_width{0};
    int image_height{0};
    std::vector<GroundProjectionEntry> entries;

    bool empty() const { return entries.empty(); }
    std::size_t size() const { return entries.size(); }
};

GroundProjectionLUT build_ground_projection_lut(
    const CameraModel& camera_model,
    int pixel_step,
    int image_width,
    int image_height);

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& lut,
    uint8_t mask_threshold,
    std::size_t max_points);

Eigen::Matrix3d rotation_matrix_from_rpy(double roll, double pitch, double yaw);

}  // namespace localization
