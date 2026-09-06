#include "lane_line_publisher/ground_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

#include <camera_utility/camera_utility.hpp>

namespace lane_line_publisher
{

std::vector<LineComponent> split_line_components(
    const cv::Mat& skeleton_mask, const int min_component_pixels)
{
    cv::Mat labels, stats, centroids;
    const int num_labels = cv::connectedComponentsWithStats(
        skeleton_mask, labels, stats, centroids, 8, CV_32S);

    std::vector<LineComponent> components;
    for (int label = 1; label < num_labels; ++label) {
        const int pixel_count = stats.at<int>(label, cv::CC_STAT_AREA);
        if (pixel_count < min_component_pixels) {
            continue;
        }
        cv::Mat component_mask = cv::Mat::zeros(skeleton_mask.size(), CV_8UC1);
        component_mask.setTo(255, labels == label);
        components.push_back(LineComponent{std::move(component_mask), pixel_count});
    }
    return components;
}

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

GroundProjectionLUT ground_projection_look_up_table(
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera)
{
    const Eigen::Vector2d invalid(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

    std::vector<std::vector<Eigen::Vector2d>> rows(static_cast<std::size_t>(intrinsics.height));
    int row_begin = intrinsics.height;
    int row_end = 0;

    for (int row = 0; row < intrinsics.height; ++row) {
        rows[static_cast<std::size_t>(row)].assign(static_cast<std::size_t>(intrinsics.width), invalid);
        for (int col = 0; col < intrinsics.width; ++col) {
            tf2::Vector3 base_point;
            if (!camera_utility::pixelToPoint(
                    cv::Point2f(static_cast<float>(col), static_cast<float>(row)),
                    intrinsics, base_T_camera, base_point))
            {
                continue;
            }

            const double distance = (base_point - base_T_camera.getOrigin()).length();
            if (distance > 10.0) {
                continue;
            }

            rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                Eigen::Vector2d(base_point.x(), base_point.y());
            row_begin = std::min(row_begin, row);
            row_end = std::max(row_end, row + 1);
        }
    }

    GroundProjectionLUT look_up_table;
    look_up_table.width = intrinsics.width;
    look_up_table.height = intrinsics.height;
    look_up_table.row_begin = (row_begin <= row_end) ? row_begin : 0;
    look_up_table.row_end = (row_begin <= row_end) ? row_end : 0;
    look_up_table.points.reserve(
        static_cast<std::size_t>(look_up_table.row_end - look_up_table.row_begin) *
        static_cast<std::size_t>(look_up_table.width));
    for (int row = look_up_table.row_begin; row < look_up_table.row_end; ++row) {
        for (int col = 0; col < look_up_table.width; ++col) {
            look_up_table.points.push_back(
                rows[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
        }
    }
    return look_up_table;
}

std::vector<Eigen::Vector2d> lane_pixels_to_base_points(
    const cv::Mat& skeleton_mask,
    const GroundProjectionLUT& look_up_table)
{
    std::vector<Eigen::Vector2d> base_points;
    if (look_up_table.row_begin >= look_up_table.row_end) {
        return base_points;
    }

    const cv::Mat band = skeleton_mask(
        cv::Range(look_up_table.row_begin, look_up_table.row_end), cv::Range::all());
    std::vector<cv::Point> nonzero;
    cv::findNonZero(band, nonzero);

    base_points.reserve(nonzero.size());
    for (const auto& pixel : nonzero) {
        Eigen::Vector2d point;
        if (!look_up_table.try_get(pixel.y + look_up_table.row_begin, pixel.x, point)) {
            continue;
        }
        base_points.push_back(point);
    }
    return base_points;
}

std::vector<Eigen::Vector2d> voxel_downsample(
    const std::vector<Eigen::Vector2d>& points,
    const double voxel_size_m)
{
    struct Voxel
    {
        Eigen::Vector2d sum = Eigen::Vector2d::Zero();
        std::size_t count = 0U;
    };

    std::map<std::pair<std::int64_t, std::int64_t>, Voxel> voxels;
    for (const auto& point : points) {
        if (!point.allFinite()) {
            continue;
        }
        const auto x_index = static_cast<std::int64_t>(std::floor(point.x() / voxel_size_m));
        const auto y_index = static_cast<std::int64_t>(std::floor(point.y() / voxel_size_m));
        auto& voxel = voxels[{x_index, y_index}];
        voxel.sum += point;
        ++voxel.count;
    }

    std::vector<Eigen::Vector2d> downsampled_points;
    downsampled_points.reserve(voxels.size());
    for (const auto& [index, voxel] : voxels) {
        (void)index;
        downsampled_points.push_back(voxel.sum / static_cast<double>(voxel.count));
    }
    return downsampled_points;
}

}
