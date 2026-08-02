#include "lane_line_publisher/ground_projection.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <camera_utility/ground_conversion.hpp>

namespace lane_line_publisher
{

bool GroundProjectionLUT::try_get(int row, int col, Eigen::Vector2d& out) const
{
    if (row < row_begin || row >= row_end || col < 0 || col >= width) {
        return false;
    }
    const auto& point = points[
        static_cast<std::size_t>(row - row_begin) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(col)];
    if (!point.allFinite()) {
        return false;
    }
    out = point;
    return true;
}

GroundProjectionLUT build_ground_projection_lut(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    const double ground_z,
    const double max_ground_intersection_distance)
{
    if (!(max_ground_intersection_distance > 0.0)) {
        throw std::invalid_argument("max_ground_intersection_distance must be greater than 0");
    }
    if (intrinsics.width <= 0 || intrinsics.height <= 0) {
        throw std::invalid_argument("image dimensions must be greater than 0");
    }

    const Eigen::Vector2d invalid(
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

    std::vector<std::vector<Eigen::Vector2d>> rows(static_cast<std::size_t>(intrinsics.height));
    int row_begin = intrinsics.height;
    int row_end = 0;

    for (int row = 0; row < intrinsics.height; ++row) {
        rows[static_cast<std::size_t>(row)].assign(static_cast<std::size_t>(intrinsics.width), invalid);
        for (int col = 0; col < intrinsics.width; ++col) {
            tf2::Vector3 base_point;
            if (!camera_utility::pixelToPoint(
                    cv::Point2f(static_cast<float>(col), static_cast<float>(row)),
                    intrinsics, base_T_camera, base_point, ground_z))
            {
                continue;
            }

            const double distance = (base_point - base_T_camera.getOrigin()).length();
            if (distance > max_ground_intersection_distance) {
                continue;
            }

            rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                Eigen::Vector2d(base_point.x(), base_point.y());
            row_begin = std::min(row_begin, row);
            row_end = std::max(row_end, row + 1);
        }
    }

    GroundProjectionLUT lut;
    lut.width = intrinsics.width;
    lut.height = intrinsics.height;
    lut.row_begin = (row_begin <= row_end) ? row_begin : 0;
    lut.row_end = (row_begin <= row_end) ? row_end : 0;
    lut.points.reserve(
        static_cast<std::size_t>(lut.row_end - lut.row_begin) * static_cast<std::size_t>(lut.width));
    for (int row = lut.row_begin; row < lut.row_end; ++row) {
        for (int col = 0; col < lut.width; ++col) {
            lut.points.push_back(rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
        }
    }
    return lut;
}

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& lut,
    const std::size_t max_points)
{
    std::vector<Eigen::Vector2d> base_points;
    if (lut.row_begin >= lut.row_end) {
        return base_points;
    }

    const cv::Mat band = skeleton_mask(cv::Range(lut.row_begin, lut.row_end), cv::Range::all());
    std::vector<cv::Point> nonzero;
    cv::findNonZero(band, nonzero);

    base_points.reserve(std::min(max_points, nonzero.size()));
    // findNonZeroは行優先(遠→近)順で返るため、末尾(近距離)から辿ってキャップする。
    for (auto it = nonzero.rbegin(); it != nonzero.rend(); ++it) {
        Eigen::Vector2d point;
        if (!lut.try_get(it->y + lut.row_begin, it->x, point)) {
            continue;
        }
        base_points.push_back(point);
        if (base_points.size() >= max_points) {
            break;
        }
    }
    return base_points;
}

}
