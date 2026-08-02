#pragma once

#include <vector>

#include <Eigen/Core>

namespace lane_line_publisher
{

std::vector<std::vector<Eigen::Vector2d>> cluster_points_into_lines(
    const std::vector<Eigen::Vector2d>& points,
    double max_link_distance_m);

std::vector<Eigen::Vector2d> resample_polyline_by_distance(
    const std::vector<Eigen::Vector2d>& ordered_points,
    double interval_m);

std::vector<Eigen::Vector2d> resample_lane_points(
    const std::vector<Eigen::Vector2d>& points,
    double max_link_distance_m,
    double interval_m);

}
