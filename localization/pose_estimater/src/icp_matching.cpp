#include "pose_estimater/icp_matching.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include <Eigen/Eigenvalues>

namespace pose_estimater
{
namespace
{

constexpr double DEGENERATE_EIGENVALUE_RATIO = 1e-3;

}

IcpTargetMap::IcpTargetMap(std::vector<IcpMapPoint> points)
: points_(std::move(points)),
  root_index_(-1)
{
    if (points_.empty()) {
        return;
    }

    std::vector<std::size_t> indices(points_.size());
    std::iota(indices.begin(), indices.end(), 0U);
    nodes_.reserve(points_.size());
    root_index_ = build_tree(indices, 0U, indices.size(), 0);
}

bool IcpTargetMap::empty() const
{
    return points_.empty();
}

const IcpMapPoint& IcpTargetMap::point(const std::size_t index) const
{
    return points_[index];
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
            return points_[lhs].position[axis] < points_[rhs].position[axis];
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
    std::size_t& nearest_index) const
{
    if (root_index_ < 0) {
        return false;
    }

    bool found = false;
    double nearest_distance_sq = max_distance_sq;
    nearest_recursive(root_index_, query, nearest_index, nearest_distance_sq, found);
    return found;
}

void IcpTargetMap::nearest_recursive(
    const int node_index,
    const Eigen::Vector2d& query,
    std::size_t& nearest_index,
    double& nearest_distance_sq,
    bool& found) const
{
    if (node_index < 0) {
        return;
    }

    const auto& node = nodes_[static_cast<std::size_t>(node_index)];
    const Eigen::Vector2d& point = points_[node.point_index].position;
    const double distance_sq = (point - query).squaredNorm();
    if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_index = node.point_index;
        found = true;
    }

    const double axis_delta = query[node.axis] - point[node.axis];
    const int near_child = axis_delta < 0.0 ? node.left : node.right;
    const int far_child = axis_delta < 0.0 ? node.right : node.left;

    nearest_recursive(near_child, query, nearest_index, nearest_distance_sq, found);
    if (axis_delta * axis_delta <= nearest_distance_sq) {
        nearest_recursive(far_child, query, nearest_index, nearest_distance_sq, found);
    }
}

IcpMatcher::IcpMatcher(const IcpConfig& config)
: config_(config)
{
}

IcpResult IcpMatcher::align_translation_only(
    const std::vector<Eigen::Vector2d>& source_points,
    const IcpTargetMap& target_map) const
{
    IcpResult result;
    if (source_points.empty() || target_map.empty()) {
        return result;
    }

    const double max_distance_sq =
        config_.max_correspondence_distance * config_.max_correspondence_distance;

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
        Eigen::Matrix2d normal_matrix = Eigen::Matrix2d::Zero();
        Eigen::Vector2d normal_residual = Eigen::Vector2d::Zero();
        std::size_t correspondences = 0U;
        double error_sum = 0.0;

        for (const auto& source_point : source_points) {
            const Eigen::Vector2d transformed_source = source_point + result.translation;
            std::size_t nearest_index = 0U;
            if (target_map.nearest(transformed_source, max_distance_sq, nearest_index)) {
                const auto& map_point = target_map.point(nearest_index);
                const double normal_error =
                    map_point.normal.dot(map_point.position - transformed_source);
                normal_matrix += map_point.normal * map_point.normal.transpose();
                normal_residual += map_point.normal * normal_error;
                error_sum += std::abs(normal_error);
                ++correspondences;
            }
        }

        result.correspondences = correspondences;
        result.mean_error =
            correspondences == 0U ? 0.0 : error_sum / static_cast<double>(correspondences);
        result.normal_matrix = normal_matrix;
        if (correspondences < config_.min_correspondences) {
            return result;
        }

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(normal_matrix);
        const double eigenvalue_threshold =
            DEGENERATE_EIGENVALUE_RATIO * static_cast<double>(correspondences);
        Eigen::Vector2d correction = Eigen::Vector2d::Zero();
        for (int i = 0; i < 2; ++i) {
            if (solver.eigenvalues()(i) > eigenvalue_threshold) {
                const Eigen::Vector2d direction = solver.eigenvectors().col(i);
                correction += (direction.dot(normal_residual) / solver.eigenvalues()(i)) * direction;
            }
        }

        result.translation += correction;
        if (correction.norm() < config_.convergence_translation_epsilon) {
            result.converged = result.mean_error <= config_.max_mean_error;
            return result;
        }
    }

    return result;
}

}
