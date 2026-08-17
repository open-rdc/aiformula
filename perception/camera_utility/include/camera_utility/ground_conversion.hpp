#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

#include "camera_utility/camera_intrinsics.hpp"

namespace camera_utility
{

tf2::Transform makeTf2Transform(
    const tf2::Vector3& position,
    double roll_deg,
    double pitch_deg,
    double yaw_deg);

bool pixelToPoint(
    const cv::Point2f& pixel,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    tf2::Vector3& base_point,
    double ground_z = 0.0);

std::vector<tf2::Vector3> pixelsToPoints(
    const std::vector<cv::Point2f>& pixels,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    double ground_z = 0.0);

bool pointToPixel(
    const tf2::Vector3& base_point,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    cv::Point2f& pixel);

}
