#include <gtest/gtest.h>

#include <vector>

#include "pose_estimater/particle_filter.hpp"

using pose_estimater::PfMapPoint;
using pose_estimater::PfTargetMap;

TEST(PfTargetMapTest, NearestFindsClosestPointWithinRadius)
{
    const std::vector<PfMapPoint> points{
        PfMapPoint{Eigen::Vector2d(0.0, 0.0)},
        PfMapPoint{Eigen::Vector2d(5.0, 0.0)},
        PfMapPoint{Eigen::Vector2d(10.0, 0.0)},
    };
    const PfTargetMap map(points);

    std::size_t nearest_index = 0U;
    EXPECT_TRUE(map.nearest(Eigen::Vector2d(4.5, 0.1), 4.0, nearest_index));
    EXPECT_EQ(nearest_index, 1U);

    EXPECT_FALSE(map.nearest(Eigen::Vector2d(100.0, 100.0), 4.0, nearest_index));
}

TEST(PfTargetMapTest, EmptyMapReportsEmpty)
{
    const PfTargetMap map(std::vector<PfMapPoint>{});

    EXPECT_TRUE(map.empty());
    std::size_t nearest_index = 0U;
    EXPECT_FALSE(map.nearest(Eigen::Vector2d(0.0, 0.0), 1.0, nearest_index));
}

using pose_estimater::Particle;
using pose_estimater::ParticleFilter;
using pose_estimater::ParticleFilterConfig;

namespace
{

ParticleFilterConfig make_default_config()
{
    ParticleFilterConfig config;
    config.num_particles = 10U;
    config.process_position_noise_std_per_m = 0.05;
    config.process_position_noise_std_per_s = 0.02;
    config.process_yaw_noise_std_per_rad = 0.05;
    config.process_yaw_noise_std_per_s = 0.01;
    config.likelihood_sigma_m = 0.3;
    config.max_correspondence_distance = 1.5;
    config.resample_ess_ratio_threshold = 0.5;
    config.reinit_ess_ratio_threshold = 0.1;
    config.reinit_consecutive_frames = 5;
    return config;
}

}  // namespace

TEST(ParticleFilterTest, InitializeSamplesAroundSeedPoseWithinExpectedSpread)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2000U;
    ParticleFilter filter(config, 7U);

    EXPECT_FALSE(filter.initialized());
    filter.initialize(10.0, -5.0, 0.5, /*position_std=*/1.0, /*yaw_std=*/0.1);
    EXPECT_TRUE(filter.initialized());

    const auto& particles = filter.particles();
    ASSERT_EQ(particles.size(), 2000U);

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_weight = 0.0;
    for (const auto& particle : particles) {
        sum_x += particle.x;
        sum_y += particle.y;
        sum_weight += particle.weight;
        EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 2000.0);
    }

    // N=2000, std=1.0 の標準誤差は約0.022。5シグマ相当の余裕を持たせる。
    EXPECT_NEAR(sum_x / particles.size(), 10.0, 0.15);
    EXPECT_NEAR(sum_y / particles.size(), -5.0, 0.15);
    EXPECT_NEAR(sum_weight, 1.0, 1e-9);
}

TEST(ParticleFilterTest, PredictAppliesUnicycleMotionWithZeroNoise)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 1U;
    config.process_position_noise_std_per_m = 0.0;
    config.process_position_noise_std_per_s = 0.0;
    config.process_yaw_noise_std_per_rad = 0.0;
    config.process_yaw_noise_std_per_s = 0.0;
    ParticleFilter filter(config, 3U);
    filter.set_particles_for_test({Particle{0.0, 0.0, 0.0, 1.0}});

    filter.predict(/*linear_velocity=*/2.0, /*yaw_rate=*/0.5, /*dt=*/0.1);

    const auto& particles = filter.particles();
    ASSERT_EQ(particles.size(), 1U);
    EXPECT_NEAR(particles[0].x, 0.2, 1e-9);
    EXPECT_NEAR(particles[0].y, 0.0, 1e-9);
    EXPECT_NEAR(particles[0].yaw, 0.05, 1e-9);
}

TEST(ParticleFilterTest, PredictNoiseProducesExpectedSpread)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2000U;
    config.process_position_noise_std_per_m = 0.0;
    config.process_position_noise_std_per_s = 0.5;  // dt=0.1 -> std=0.05
    config.process_yaw_noise_std_per_rad = 0.0;
    config.process_yaw_noise_std_per_s = 0.0;
    ParticleFilter filter(config, 11U);

    std::vector<Particle> particles(2000U, Particle{0.0, 0.0, 0.0, 1.0 / 2000.0});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/0.0, /*yaw_rate=*/0.0, /*dt=*/0.1);

    double sum_sq = 0.0;
    for (const auto& particle : filter.particles()) {
        sum_sq += particle.x * particle.x;
    }
    const double sample_std = std::sqrt(sum_sq / filter.particles().size());
    const double expected_std = 0.05;

    EXPECT_NEAR(sample_std, expected_std, expected_std * 0.3);
}
