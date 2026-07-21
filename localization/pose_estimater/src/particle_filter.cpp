#include "pose_estimater/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace pose_estimater
{

namespace
{

double sample_gaussian(std::mt19937& rng, const double std_dev)
{
    if (std_dev <= 0.0) {
        return 0.0;
    }
    std::normal_distribution<double> distribution(0.0, std_dev);
    return distribution(rng);
}

double normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}  // namespace

PfTargetMap::PfTargetMap(std::vector<PfMapPoint> points)
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

bool PfTargetMap::empty() const
{
    return points_.empty();
}

const PfMapPoint& PfTargetMap::point(const std::size_t index) const
{
    return points_[index];
}

int PfTargetMap::build_tree(
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

bool PfTargetMap::nearest(
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

void PfTargetMap::nearest_recursive(
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

ParticleFilter::ParticleFilter(const ParticleFilterConfig& config, const std::uint32_t seed)
: config_(config),
  rng_(seed)
{
}

bool ParticleFilter::initialized() const
{
    return initialized_;
}

void ParticleFilter::initialize(
    const double x, const double y, const double yaw,
    const double position_std, const double yaw_std)
{
    const double uniform_weight = 1.0 / static_cast<double>(config_.num_particles);
    particles_.assign(config_.num_particles, Particle{});
    for (auto& particle : particles_) {
        particle.x = x + sample_gaussian(rng_, position_std);
        particle.y = y + sample_gaussian(rng_, position_std);
        particle.yaw = normalize_angle(yaw + sample_gaussian(rng_, yaw_std));
        particle.weight = uniform_weight;
    }
    initialized_ = true;
    low_ess_streak_ = 0;
}

const std::vector<Particle>& ParticleFilter::particles() const
{
    return particles_;
}

void ParticleFilter::set_particles_for_test(std::vector<Particle> particles)
{
    particles_ = std::move(particles);
    initialized_ = true;
}

void ParticleFilter::predict(const double linear_velocity, const double yaw_rate, const double dt)
{
    const double position_noise_std =
        config_.process_position_noise_std_per_m * std::abs(linear_velocity) * dt +
        config_.process_position_noise_std_per_s * dt;
    const double yaw_noise_std =
        config_.process_yaw_noise_std_per_rad * std::abs(yaw_rate) * dt +
        config_.process_yaw_noise_std_per_s * dt;

    for (auto& particle : particles_) {
        const double dx = linear_velocity * dt * std::cos(particle.yaw);
        const double dy = linear_velocity * dt * std::sin(particle.yaw);
        particle.x += dx + sample_gaussian(rng_, position_noise_std);
        particle.y += dy + sample_gaussian(rng_, position_noise_std);
        particle.yaw = normalize_angle(particle.yaw + yaw_rate * dt + sample_gaussian(rng_, yaw_noise_std));
    }
}

}
