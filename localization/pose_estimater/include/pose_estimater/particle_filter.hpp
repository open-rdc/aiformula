#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace pose_estimater {

struct PfMapPoint {
    Eigen::Vector2d position;
};

class PfTargetMap {
   public:
    explicit PfTargetMap (std::vector<PfMapPoint> points);

    bool empty () const;

    bool nearest (const Eigen::Vector2d &query, double max_distance_sq, std::size_t &nearest_index) const;

    const PfMapPoint &point (std::size_t index) const;

   private:
    struct KdNode {
        std::size_t point_index;
        int         left;
        int         right;
        int         axis;
    };

    int  build_tree (std::vector<std::size_t> &indices, std::size_t begin, std::size_t end, int depth);
    void nearest_recursive (int node_index, const Eigen::Vector2d &query, std::size_t &nearest_index, double &nearest_distance_sq, bool &found) const;

    std::vector<PfMapPoint> points_;
    std::vector<KdNode>     nodes_;
    int                     root_index_;
};

struct ParticleFilterConfig {
    std::size_t num_particles = 0U;
    //   std_fw  = sqrt(ff^2*|Δs| + fr^2*|Δθ|), std_rot = sqrt(rf^2*|Δs| + rr^2*|Δθ|)
    double odom_fw_dev_per_fw           = 0.0;
    double odom_fw_dev_per_rot          = 0.0;
    double odom_rot_dev_per_fw          = 0.0;
    double odom_rot_dev_per_rot         = 0.0;
    double likelihood_dev               = 0.0;
    double likelihood_max_dist          = 0.0;
    double resample_ess_ratio_threshold = 0.0;
    double reinit_residual_threshold = 0.0;
    int    reinit_consecutive_frames = 0;
    double min_position_variance = 0.0;
    double min_yaw_variance      = 0.0;
};

struct Particle {
    double x      = 0.0;
    double y      = 0.0;
    double yaw    = 0.0;
    double weight = 0.0;
};

struct PoseEstimate2D {
    double          x                   = 0.0;
    double          y                   = 0.0;
    double          yaw                 = 0.0;
    Eigen::Matrix2d position_covariance = Eigen::Matrix2d::Zero ();
    double          yaw_variance        = 0.0;
};

class ParticleFilter {
   public:
    ParticleFilter (const ParticleFilterConfig &config, std::uint32_t seed);

    bool initialized () const;
    void initialize (double x, double y, double yaw, double position_std, double yaw_std);
    void predict (double linear_velocity, double yaw_rate, double dt);
    void update_weights (const std::vector<Eigen::Vector2d> &source_points_base_link, const PfTargetMap &target_map);
    double effective_sample_size_ratio () const;
    bool should_resample () const;
    void resample ();
    bool needs_reinitialization () const;
    PoseEstimate2D estimate () const;
    const std::vector<Particle> &particles () const;
    void set_particles_for_test (std::vector<Particle> particles);

   private:
    ParticleFilterConfig  config_;
    std::vector<Particle> particles_;
    std::mt19937          rng_;
    bool                  initialized_ = false;
    int                   lost_streak_ = 0;
};

}  // namespace pose_estimater
