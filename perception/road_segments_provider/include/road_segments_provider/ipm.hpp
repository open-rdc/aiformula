#pragma once

#include <array>
#include <optional>

#include <geometry_msgs/msg/point.hpp>

namespace road_segments_provider
{

struct CameraParams
{
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    // Rotation matrix: camera_frame -> base_link, row-major [r00,r01,r02, r10,...,r22]
    std::array<double, 9> R{};
    // Translation: camera origin expressed in base_link frame [tx, ty, tz]
    std::array<double, 3> t{};
};

// Converts a quaternion (qx, qy, qz, qw) to a row-major 3x3 rotation matrix.
std::array<double, 9> quat_to_rotation_matrix(double qx, double qy, double qz, double qw);

// Projects pixel (u, v) onto the ground plane (z = 0 in base_link frame).
// Returns nullopt when the ray points away from the ground or is parallel to it.
std::optional<geometry_msgs::msg::Point> project_pixel_to_ground(
    int u, int v, const CameraParams & params);

}  // namespace road_segments_provider
