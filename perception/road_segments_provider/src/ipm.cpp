#include "road_segments_provider/ipm.hpp"

#include <cmath>

namespace road_segments_provider
{

std::array<double, 9> quat_to_rotation_matrix(double qx, double qy, double qz, double qw)
{
    std::array<double, 9> R{};
    R[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    R[1] = 2.0 * (qx * qy - qz * qw);
    R[2] = 2.0 * (qx * qz + qy * qw);
    R[3] = 2.0 * (qx * qy + qz * qw);
    R[4] = 1.0 - 2.0 * (qx * qx + qz * qz);
    R[5] = 2.0 * (qy * qz - qx * qw);
    R[6] = 2.0 * (qx * qz - qy * qw);
    R[7] = 2.0 * (qy * qz + qx * qw);
    R[8] = 1.0 - 2.0 * (qx * qx + qy * qy);
    return R;
}

std::optional<geometry_msgs::msg::Point> project_pixel_to_ground(
    int u, int v, const CameraParams & params)
{
    // Ray direction in camera frame (unnormalized)
    const double dx = (static_cast<double>(u) - params.cx) / params.fx;
    const double dy = (static_cast<double>(v) - params.cy) / params.fy;
    const double dz = 1.0;

    // Transform ray direction to base_link frame: dir_base = R * [dx, dy, dz]
    const auto & R = params.R;
    const double bx = R[0] * dx + R[1] * dy + R[2] * dz;
    const double by = R[3] * dx + R[4] * dy + R[5] * dz;
    const double bz = R[6] * dx + R[7] * dy + R[8] * dz;

    // Ground intersection: t + lambda * [bx, by, bz], z-component = 0
    // params.t[2] + lambda * bz = 0  =>  lambda = -t[2] / bz
    if (std::abs(bz) < 1e-9) {
        return std::nullopt;
    }

    const double lambda = -params.t[2] / bz;
    if (lambda < 0.0) {
        // Intersection is behind the camera
        return std::nullopt;
    }

    geometry_msgs::msg::Point p;
    p.x = params.t[0] + lambda * bx;
    p.y = params.t[1] + lambda * by;
    p.z = 0.0;
    return p;
}

}  // namespace road_segments_provider
