#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

#include <camera_utility/camera_intrinsics.hpp>

namespace lane_line_publisher
{

struct GroundProjectionEntry
{
    int row;
    int col;
    float x_base;
    float y_base;
};

struct GroundProjectionLUT
{
    std::vector<GroundProjectionEntry> entries;

    bool empty() const { return entries.empty(); }
    std::size_t size() const { return entries.size(); }
};

GroundProjectionLUT build_ground_projection_lut(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    double ground_z,
    double min_ground_intersection_distance,
    double max_ground_intersection_distance,
    int pixel_step);

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& lut,
    uint8_t mask_threshold,
    std::size_t max_points);

}
