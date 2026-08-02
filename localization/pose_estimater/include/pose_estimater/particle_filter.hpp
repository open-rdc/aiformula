#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Core>

namespace pose_estimater
{

struct PfMapPoint
{
    Eigen::Vector2d position;
};

class PfTargetMap
{
public:
    explicit PfTargetMap(std::vector<PfMapPoint> points);

    bool empty() const;

    bool nearest(
        const Eigen::Vector2d& query,
        double max_distance_sq,
        std::size_t& nearest_index) const;

    const PfMapPoint& point(std::size_t index) const;

private:
    struct KdNode
    {
        std::size_t point_index;
        int left;
        int right;
        int axis;
    };

    int build_tree(std::vector<std::size_t>& indices, std::size_t begin, std::size_t end, int depth);
    void nearest_recursive(
        int node_index,
        const Eigen::Vector2d& query,
        std::size_t& nearest_index,
        double& nearest_distance_sq,
        bool& found) const;

    std::vector<PfMapPoint> points_;
    std::vector<KdNode> nodes_;
    int root_index_;
};

struct ParticleFilterConfig
{
    std::size_t num_particles = 0U;
    double process_position_noise_std_per_m = 0.0;
    double process_position_noise_std_per_s = 0.0;
    double process_yaw_noise_std_per_rad = 0.0;
    double process_yaw_noise_std_per_s = 0.0;
    double likelihood_sigma_m = 0.0;
    double max_correspondence_distance = 0.0;
    double resample_ess_ratio_threshold = 0.0;
    // 最良パーティクルのRMS残差がこの値を超えたフレームを見失いとして数える。
    double reinit_residual_threshold_m = 0.0;
    int reinit_consecutive_frames = 0;
    // estimate() が返す共分散の下限。リサンプルで粒子が潰れても
    // 実際の推定誤差より小さい共分散を公表しないようにする。
    double min_position_variance = 0.0;
    double min_yaw_variance = 0.0;
};

struct Particle
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double weight = 0.0;
};

struct PoseEstimate2D
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    Eigen::Matrix2d position_covariance = Eigen::Matrix2d::Zero();
    double yaw_variance = 0.0;
};

class ParticleFilter
{
public:
    ParticleFilter(const ParticleFilterConfig& config, std::uint32_t seed);

    bool initialized() const;

    void initialize(double x, double y, double yaw, double position_std, double yaw_std);

    void predict(double linear_velocity, double yaw_rate, double dt);

    // 重み計算に加え、連続見失いフレーム数(見失いストリーク)を内部で更新する。
    void update_weights(
        const std::vector<Eigen::Vector2d>& source_points_base_link,
        const PfTargetMap& target_map);

    double effective_sample_size_ratio() const;

    bool should_resample() const;

    void resample();

    // update_weights() で得た最良パーティクルのRMS残差が reinit_residual_threshold_m を
    // reinit_consecutive_frames 回連続で上回ったら true。initialize() でリセットされる。
    bool needs_reinitialization() const;

    PoseEstimate2D estimate() const;

    const std::vector<Particle>& particles() const;

    // テスト専用の抜け道: 乱数サンプリングを経由せずパーティクル集合を直接置き換え、
    // initialized() を true にする。本番コードは常に initialize()/predict()/resample() を経由する。
    void set_particles_for_test(std::vector<Particle> particles);

private:
    ParticleFilterConfig config_;
    std::vector<Particle> particles_;
    std::mt19937 rng_;
    bool initialized_ = false;
    int lost_streak_ = 0;
};

}
