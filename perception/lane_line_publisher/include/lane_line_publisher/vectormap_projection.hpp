#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <camera_utility/camera_intrinsics.hpp>

namespace lane_line_publisher
{

struct ProjectedLineString
{
    std::vector<cv::Point2f> pixels;
    cv::Scalar color_bgr;
    float thickness_px;
};

std::vector<ProjectedLineString> project_vector_map_markers(
    const visualization_msgs::msg::MarkerArray& marker_array,
    const tf2::Transform& base_T_map,
    const tf2::Transform& base_T_camera,
    const camera_utility::CameraIntrinsics& intrinsics);

void draw_projected_line_strings(
    cv::Mat& image,
    const std::vector<ProjectedLineString>& line_strings);

}
