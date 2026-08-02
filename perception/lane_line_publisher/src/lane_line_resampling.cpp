#include "lane_line_publisher/lane_line_resampling.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace lane_line_publisher
{
namespace
{

std::optional<std::size_t> find_nearest_unvisited(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<bool>& visited,
    const Eigen::Vector2d& from,
    const double max_link_distance_m)
{
    std::optional<std::size_t> nearest;
    double nearest_distance_sq = max_link_distance_m * max_link_distance_m;
    for (std::size_t i = 0U; i < points.size(); ++i) {
        if (visited[i]) {
            continue;
        }
        const double distance_sq = (points[i] - from).squaredNorm();
        if (distance_sq <= nearest_distance_sq) {
            nearest_distance_sq = distance_sq;
            nearest = i;
        }
    }
    return nearest;
}

}  // namespace

std::vector<std::vector<Eigen::Vector2d>> cluster_points_into_lines(
    const std::vector<Eigen::Vector2d>& points,
    const double max_link_distance_m)
{
    std::vector<bool> visited(points.size(), false);
    std::vector<std::vector<Eigen::Vector2d>> lines;

    for (std::size_t seed = 0U; seed < points.size(); ++seed) {
        if (visited[seed]) {
            continue;
        }
        visited[seed] = true;

        std::vector<Eigen::Vector2d> forward{points[seed]};
        std::size_t current = seed;
        while (const auto next = find_nearest_unvisited(points, visited, points[current], max_link_distance_m)) {
            visited[*next] = true;
            forward.push_back(points[*next]);
            current = *next;
        }

        std::vector<Eigen::Vector2d> backward;
        current = seed;
        while (const auto next = find_nearest_unvisited(points, visited, points[current], max_link_distance_m)) {
            visited[*next] = true;
            backward.push_back(points[*next]);
            current = *next;
        }
        std::reverse(backward.begin(), backward.end());

        backward.insert(backward.end(), forward.begin(), forward.end());
        lines.push_back(std::move(backward));
    }

    return lines;
}

std::vector<Eigen::Vector2d> resample_polyline_by_distance(
    const std::vector<Eigen::Vector2d>& ordered_points,
    const double interval_m)
{
    if (!(interval_m > 0.0) || !std::isfinite(interval_m)) {
        return ordered_points;
    }
    if (ordered_points.size() < 2U) {
        return ordered_points;
    }

    std::vector<Eigen::Vector2d> resampled;
    resampled.push_back(ordered_points.front());

    double distance_to_next_sample = interval_m;
    for (std::size_t i = 1U; i < ordered_points.size(); ++i) {
        const Eigen::Vector2d& segment_start = ordered_points[i - 1U];
        const Eigen::Vector2d& segment_end = ordered_points[i];
        const double segment_length = (segment_end - segment_start).norm();
        if (segment_length <= 0.0) {
            continue;
        }
        const Eigen::Vector2d direction = (segment_end - segment_start) / segment_length;

        double position_in_segment = 0.0;
        while (distance_to_next_sample <= (segment_length - position_in_segment)) {
            position_in_segment += distance_to_next_sample;
            resampled.push_back(segment_start + direction * position_in_segment);
            distance_to_next_sample = interval_m;
        }
        distance_to_next_sample -= (segment_length - position_in_segment);
    }

    const Eigen::Vector2d& last_point = ordered_points.back();
    if ((resampled.back() - last_point).norm() > 1.0e-9) {
        resampled.push_back(last_point);
    }
    return resampled;
}

std::vector<Eigen::Vector2d> resample_lane_points(
    const std::vector<Eigen::Vector2d>& points,
    const double max_link_distance_m,
    const double interval_m)
{
    const auto lines = cluster_points_into_lines(points, max_link_distance_m);
    std::vector<Eigen::Vector2d> resampled;
    for (const auto& line : lines) {
        const auto line_resampled = resample_polyline_by_distance(line, interval_m);
        resampled.insert(resampled.end(), line_resampled.begin(), line_resampled.end());
    }
    return resampled;
}

}
