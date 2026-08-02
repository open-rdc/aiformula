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
    lost_streak_ = 0;
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

double ParticleFilter::effective_sample_size_ratio() const
{
    if (particles_.empty()) {
        return 0.0;
    }

    double sum_sq = 0.0;
    for (const auto& particle : particles_) {
        sum_sq += particle.weight * particle.weight;
    }
    if (sum_sq <= 0.0) {
        return 0.0;
    }

    const double effective_sample_size = 1.0 / sum_sq;
    return effective_sample_size / static_cast<double>(particles_.size());
}

bool ParticleFilter::should_resample() const
{
    return effective_sample_size_ratio() < config_.resample_ess_ratio_threshold;
}

void ParticleFilter::resample()
{
    const std::size_t n = particles_.size();
    if (n == 0U) {
        return;
    }

    std::vector<double> cumulative_weight(n);
    double running = 0.0;
    for (std::size_t i = 0U; i < n; ++i) {
        running += particles_[i].weight;
        cumulative_weight[i] = running;
    }

    std::uniform_real_distribution<double> offset_distribution(0.0, 1.0 / static_cast<double>(n));
    const double start = offset_distribution(rng_);

    std::vector<Particle> resampled;
    resampled.reserve(n);
    std::size_t source_index = 0U;
    for (std::size_t i = 0U; i < n; ++i) {
        const double target = start + static_cast<double>(i) / static_cast<double>(n);
        while (source_index + 1U < n && cumulative_weight[source_index] < target) {
            ++source_index;
        }
        resampled.push_back(particles_[source_index]);
    }

    const double uniform_weight = 1.0 / static_cast<double>(n);
    for (auto& particle : resampled) {
        particle.weight = uniform_weight;
    }
    particles_ = std::move(resampled);
}

void ParticleFilter::update_weights(
    const std::vector<Eigen::Vector2d>& source_points_base_link,
    const PfTargetMap& target_map)
{
    if (particles_.empty() || source_points_base_link.empty()) {
        return;
    }

    const double max_distance_sq = config_.max_correspondence_distance * config_.max_correspondence_distance;
    const double inv_two_sigma_sq = 1.0 / (2.0 * config_.likelihood_sigma_m * config_.likelihood_sigma_m);
    const double inv_point_count = 1.0 / static_cast<double>(source_points_base_link.size());

    // 対応距離を超えた点は距離を上限で打ち切って一定の罰を与える。無罰にすると
    // 地図から外れた点が増えるほど罰の総和が減り、車線幅ぶんずれた姿勢が
    // 真値より高い尤度を得てしまう。
    // さらに観測点数で正規化し、実効的な尤度の鋭さを likelihood_sigma_m だけで
    // 決まるようにする。点数ぶん累積すると実効σが σ/√N まで縮み、
    // initialize() のばら撒き幅とスケールが合わなくなる。
    std::vector<double> log_likelihoods(particles_.size());
    for (std::size_t i = 0U; i < particles_.size(); ++i) {
        const Particle& particle = particles_[i];
        const double cos_yaw = std::cos(particle.yaw);
        const double sin_yaw = std::sin(particle.yaw);

        double residual_sq_sum = 0.0;
        for (const auto& source_point : source_points_base_link) {
            const Eigen::Vector2d transformed(
                particle.x + cos_yaw * source_point.x() - sin_yaw * source_point.y(),
                particle.y + sin_yaw * source_point.x() + cos_yaw * source_point.y());

            std::size_t nearest_index = 0U;
            if (target_map.nearest(transformed, max_distance_sq, nearest_index)) {
                residual_sq_sum +=
                    (target_map.point(nearest_index).position - transformed).squaredNorm();
            } else {
                residual_sq_sum += max_distance_sq;
            }
        }

        log_likelihoods[i] = -residual_sq_sum * inv_point_count * inv_two_sigma_sq;
    }

    const double best_log_likelihood =
        *std::max_element(log_likelihoods.begin(), log_likelihoods.end());

    // 最良値を引いてからexpするのでアンダーフローしない。最良パーティクルの係数は
    // 常に1になるため weight_sum は必ず正になる。
    double weight_sum = 0.0;
    for (std::size_t i = 0U; i < particles_.size(); ++i) {
        particles_[i].weight *= std::exp(log_likelihoods[i] - best_log_likelihood);
        weight_sum += particles_[i].weight;
    }
    for (auto& particle : particles_) {
        particle.weight /= weight_sum;
    }

    // 見失いは最良パーティクルのRMS残差で判定する。ESS比は重みの偏りしか測らないため、
    // 全パーティクルが同程度に地図から外れている状態を検出できない。
    const double best_rms_residual = std::sqrt(-best_log_likelihood / inv_two_sigma_sq);
    if (best_rms_residual > config_.reinit_residual_threshold_m) {
        ++lost_streak_;
    } else {
        lost_streak_ = 0;
    }
}

bool ParticleFilter::needs_reinitialization() const
{
    return lost_streak_ >= config_.reinit_consecutive_frames;
}

PoseEstimate2D ParticleFilter::estimate() const
{
    PoseEstimate2D result;
    if (particles_.empty()) {
        return result;
    }

    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weighted_sin = 0.0;
    double weighted_cos = 0.0;
    double weight_sum = 0.0;
    for (const auto& particle : particles_) {
        weighted_x += particle.weight * particle.x;
        weighted_y += particle.weight * particle.y;
        weighted_sin += particle.weight * std::sin(particle.yaw);
        weighted_cos += particle.weight * std::cos(particle.yaw);
        weight_sum += particle.weight;
    }
    if (weight_sum <= 0.0) {
        return result;
    }

    result.x = weighted_x / weight_sum;
    result.y = weighted_y / weight_sum;
    result.yaw = std::atan2(weighted_sin / weight_sum, weighted_cos / weight_sum);

    Eigen::Matrix2d position_covariance = Eigen::Matrix2d::Zero();
    double yaw_variance = 0.0;
    for (const auto& particle : particles_) {
        const double normalized_weight = particle.weight / weight_sum;
        const Eigen::Vector2d position_delta(particle.x - result.x, particle.y - result.y);
        position_covariance += normalized_weight * position_delta * position_delta.transpose();

        const double yaw_delta = normalize_angle(particle.yaw - result.yaw);
        yaw_variance += normalized_weight * yaw_delta * yaw_delta;
    }
    // リサンプルで粒子が潰れると分散が実際の推定誤差より桁違いに小さく出る。
    // 下限を設けて、EKF側が観測を過信して利得1で追従してしまうのを防ぐ。
    result.position_covariance = position_covariance;
    result.position_covariance(0, 0) =
        std::max(result.position_covariance(0, 0), config_.min_position_variance);
    result.position_covariance(1, 1) =
        std::max(result.position_covariance(1, 1), config_.min_position_variance);
    result.yaw_variance = std::max(yaw_variance, config_.min_yaw_variance);

    return result;
}

}
