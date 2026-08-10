#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

#include <camera_utility/camera_intrinsics.hpp>

namespace lane_line_publisher
{

struct LineComponent
{
    cv::Mat mask;
    int pixel_count;
};

std::vector<LineComponent> split_line_components(
    const cv::Mat& skeleton_mask, int min_component_pixels);

struct GroundProjectionLUT
{
    int width = 0;
    int height = 0;
    int row_begin = 0;  // 有効な行の開始
    int row_end = 0;    // 有効な行の終端
    std::vector<Eigen::Vector2d> points;  // size = (row_end-row_begin)*width, 無効セルはNaN

    bool try_get(int row, int col, Eigen::Vector2d& out) const;
};

GroundProjectionLUT ground_projection_look_up_table(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera);

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& look_up_table);

std::vector<Eigen::Vector2d> voxel_downsample(
    const std::vector<Eigen::Vector2d>& points,
    double voxel_size_m);

}
