#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

#include <camera_utility/camera_intrinsics.hpp>

namespace lane_line_publisher
{

struct GroundProjectionLUT
{
    int width = 0;
    int height = 0;
    int row_begin = 0;  // 有効な行の開始（inclusive）
    int row_end = 0;    // 有効な行の終端（exclusive）
    std::vector<Eigen::Vector2d> points;  // size = (row_end-row_begin)*width, 無効セルはNaN

    bool try_get(int row, int col, Eigen::Vector2d& out) const;
};

GroundProjectionLUT build_ground_projection_lut(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    double ground_z,
    double max_ground_intersection_distance);

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& lut,
    std::size_t max_points);

}
