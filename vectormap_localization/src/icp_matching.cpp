#include "vectormap_localization/icp_matching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vectormap_localization
{

IcpTargetMap::IcpTargetMap(std::vector<Eigen::Vector2d> points)
: points_(std::move(points)),
  root_index_(-1)
{
    if (points_.empty()) {
        return;
    }

    std::vector<std::size_t> indices;
    indices.reserve(points_.size());
    for (std::size_t i = 0U; i < points_.size(); ++i) {
        indices.push_back(i);
    }
    nodes_.reserve(points_.size());
    root_index_ = build_tree(indices, 0U, indices.size(), 0);
}

bool IcpTargetMap::empty() const
{
    return points_.empty();
}

std::size_t IcpTargetMap::size() const
{
    return points_.size();
}

const std::vector<Eigen::Vector2d>& IcpTargetMap::points() const
{
    return points_;
}

int IcpTargetMap::build_tree(
    std::vector<std::size_t>& indices,
    const std::size_t begin,
    const std::size_t end,
    const int depth)
{
    if (begin >= end) {
        return -1;
    }

    const int axis = depth % 2;
    const std::size_t middle = begin + (end - begin) / 2U;
    std::nth_element(
        indices.begin() + static_cast<std::ptrdiff_t>(begin),
        indices.begin() + static_cast<std::ptrdiff_t>(middle),
        indices.begin() + static_cast<std::ptrdiff_t>(end),
        [this, axis](const std::size_t lhs, const std::size_t rhs) {
            return points_[lhs][axis] < points_[rhs][axis];
        });

    const int node_index = static_cast<int>(nodes_.size());
    nodes_.push_back(KdNode{indices[middle], -1, -1, axis});
    nodes_[node_index].left = build_tree(indices, begin, middle, depth + 1);
    nodes_[node_index].right = build_tree(indices, middle + 1U, end, depth + 1);
    return node_index;
}

bool IcpTargetMap::nearest(
    const Eigen::Vector2d& query,
    const double max_distance_sq,
    Eigen::Vector2d& nearest_point,
    double& nearest_distance_sq) const
{
    if (root_index_ < 0 || max_distance_sq <= 0.0) {
        return false;
    }

    bool found = false;
    nearest_distance_sq = max_distance_sq;
    nearest_recursive(root_index_, query, max_distance_sq, nearest_point, nearest_distance_sq, found);
    return found;
}

void IcpTargetMap::nearest_recursive(
    const int node_index,
    const Eigen::Vector2d& query,
    const double max_distance_sq,
    Eigen::Vector2d& nearest_point,
    double& nearest_distance_sq,
    bool& found) const
{
    if (node_index < 0) {
        return;
    }

    const auto& node = nodes_[static_cast<std::size_t>(node_index)];
    const Eigen::Vector2d& point = points_[node.point_index];
    const double distance_sq = (point - query).squaredNorm();
    if (distance_sq <= max_distance_sq && distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_point = point;
        found = true;
    }

    const double axis_delta = query[node.axis] - point[node.axis];
    const int near_child = axis_delta < 0.0 ? node.left : node.right;
    const int far_child = axis_delta < 0.0 ? node.right : node.left;

    nearest_recursive(near_child, query, max_distance_sq, nearest_point, nearest_distance_sq, found);
    if (axis_delta * axis_delta <= nearest_distance_sq) {
        nearest_recursive(far_child, query, max_distance_sq, nearest_point, nearest_distance_sq, found);
    }
}

IcpMatcher::IcpMatcher(const IcpConfig& config)
: config_(config)
{
    if (config_.max_iterations <= 0) {
        throw std::invalid_argument("max_iterations must be greater than 0");
    }
    if (config_.max_correspondence_distance <= 0.0) {
        throw std::invalid_argument("max_correspondence_distance must be greater than 0");
    }
    if (config_.convergence_translation_epsilon <= 0.0) {
        throw std::invalid_argument("convergence_translation_epsilon must be greater than 0");
    }
    if (config_.min_correspondences == 0U) {
        throw std::invalid_argument("min_correspondences must be greater than 0");
    }
}

IcpResult IcpMatcher::align_translation_only(
    const std::vector<Eigen::Vector2d>& source_points,
    const IcpTargetMap& target_map) const
{
    if (source_points.empty() || target_map.empty()) {
        return {false, Eigen::Vector2d::Zero(), 0U, 0.0};
    }

    Eigen::Vector2d translation = Eigen::Vector2d::Zero();
    std::size_t last_correspondences = 0U;
    double last_mean_error = 0.0;

    const double max_distance_sq =
        config_.max_correspondence_distance * config_.max_correspondence_distance;

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
        Eigen::Vector2d correction = Eigen::Vector2d::Zero();
        std::size_t correspondences = 0U;
        double error_sum = 0.0;

        for (const auto& source_point : source_points) {
            const Eigen::Vector2d transformed_source = source_point + translation;
            Eigen::Vector2d nearest_point = Eigen::Vector2d::Zero();
            double nearest_distance_sq = std::numeric_limits<double>::max();
            if (target_map.nearest(
                    transformed_source,
                    max_distance_sq,
                    nearest_point,
                    nearest_distance_sq))
            {
                correction += nearest_point - transformed_source;
                error_sum += std::sqrt(nearest_distance_sq);
                ++correspondences;
            }
        }

        if (correspondences < config_.min_correspondences) {
            return {false, translation, correspondences, correspondences == 0U ? 0.0 : error_sum / correspondences};
        }

        correction /= static_cast<double>(correspondences);
        translation += correction;
        last_correspondences = correspondences;
        last_mean_error = error_sum / static_cast<double>(correspondences);

        if (correction.norm() < config_.convergence_translation_epsilon) {
            return {true, translation, last_correspondences, last_mean_error};
        }
    }

    return {true, translation, last_correspondences, last_mean_error};
}

}  // namespace vectormap_localization
