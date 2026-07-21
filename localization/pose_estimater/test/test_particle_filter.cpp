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

TEST(ParticleFilterTest, EffectiveSampleSizeRatioIsOneForUniformWeights)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 4U;
    ParticleFilter filter(config, 5U);
    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 0.25},
        Particle{1.0, 0.0, 0.0, 0.25},
        Particle{2.0, 0.0, 0.0, 0.25},
        Particle{3.0, 0.0, 0.0, 0.25},
    });

    EXPECT_NEAR(filter.effective_sample_size_ratio(), 1.0, 1e-9);
}

TEST(ParticleFilterTest, ShouldResampleTriggersWhenEssRatioBelowThreshold)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 3U;
    config.resample_ess_ratio_threshold = 0.5;
    ParticleFilter filter(config, 5U);
    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 0.9},
        Particle{1.0, 0.0, 0.0, 0.05},
        Particle{2.0, 0.0, 0.0, 0.05},
    });

    EXPECT_TRUE(filter.should_resample());
}

TEST(ParticleFilterTest, SystematicResamplingRespectsLowVarianceBound)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 3U;
    ParticleFilter filter(config, 42U);
    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 0.9},
        Particle{1.0, 0.0, 0.0, 0.05},
        Particle{2.0, 0.0, 0.0, 0.05},
    });

    filter.resample();

    ASSERT_EQ(filter.particles().size(), 3U);
    int count_at_0 = 0;
    int count_at_1 = 0;
    int count_at_2 = 0;
    for (const auto& particle : filter.particles()) {
        if (particle.x == 0.0) {
            ++count_at_0;
        } else if (particle.x == 1.0) {
            ++count_at_1;
        } else {
            ++count_at_2;
        }
        EXPECT_DOUBLE_EQ(particle.weight, 1.0 / 3.0);
    }

    // systematic resampling の低分散性: count は floor(N*w) か ceil(N*w) のいずれか。
    EXPECT_GE(count_at_0, 2);  // floor(0.9*3)=2
    EXPECT_LE(count_at_0, 3);  // ceil(0.9*3)=3
    EXPECT_LE(count_at_1, 1);  // ceil(0.05*3)=1
    EXPECT_LE(count_at_2, 1);  // ceil(0.05*3)=1
    EXPECT_EQ(count_at_0 + count_at_1 + count_at_2, 3);
}

TEST(ParticleFilterTest, UpdateWeightsFavorsCloserParticle)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 3U;
    config.likelihood_sigma_m = 0.3;
    config.max_correspondence_distance = 2.0;
    ParticleFilter filter(config, 5U);

    filter.set_particles_for_test({
        Particle{4.0, 0.0, 0.0, 1.0},       // (1,0)->(5,0): 完全一致
        Particle{3.0, 0.0, 0.0, 1.0},       // (1,0)->(4,0): 残差1.0m
        Particle{100.0, 100.0, 0.0, 1.0},   // 対応点なし
    });

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> source_points{Eigen::Vector2d(1.0, 0.0)};

    filter.update_weights(source_points, target_map);

    const auto& particles = filter.particles();
    EXPECT_GT(particles[0].weight, particles[1].weight);
    EXPECT_GT(particles[1].weight, particles[2].weight);
    EXPECT_NEAR(particles[2].weight, 0.0, 1e-9);

    double weight_sum = 0.0;
    for (const auto& particle : particles) {
        weight_sum += particle.weight;
    }
    EXPECT_NEAR(weight_sum, 1.0, 1e-9);
}

TEST(ParticleFilterTest, UpdateWeightsFallsBackToUniformWhenAllParticlesHaveZeroWeight)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    config.max_correspondence_distance = 0.5;
    ParticleFilter filter(config, 5U);

    filter.set_particles_for_test({
        Particle{100.0, 100.0, 0.0, 1.0},
        Particle{200.0, 200.0, 0.0, 1.0},
    });

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> source_points{Eigen::Vector2d(1.0, 0.0)};

    filter.update_weights(source_points, target_map);

    for (const auto& particle : filter.particles()) {
        EXPECT_NEAR(particle.weight, 0.5, 1e-9);
    }
}

TEST(ParticleFilterTest, NeedsReinitializationAfterConsecutiveLowEssFrames)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    config.max_correspondence_distance = 0.5;  // 対応点が絶対に見つからない設定
    config.reinit_ess_ratio_threshold = 0.5;
    config.reinit_consecutive_frames = 3;
    ParticleFilter filter(config, 5U);
    filter.set_particles_for_test({
        Particle{100.0, 100.0, 0.0, 0.5},
        Particle{200.0, 200.0, 0.0, 0.5},
    });

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> source_points{Eigen::Vector2d(1.0, 0.0)};

    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(source_points, target_map);  // streak=1（全滅フォールバック）
    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(source_points, target_map);  // streak=2
    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(source_points, target_map);  // streak=3 >= reinit_consecutive_frames
    EXPECT_TRUE(filter.needs_reinitialization());
}

TEST(ParticleFilterTest, GoodEssRatioResetsReinitStreak)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 1U;
    config.likelihood_sigma_m = 0.3;
    config.max_correspondence_distance = 2.0;
    config.reinit_ess_ratio_threshold = 0.5;
    config.reinit_consecutive_frames = 1;
    ParticleFilter filter(config, 5U);

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> matching_source{Eigen::Vector2d(1.0, 0.0)};
    const std::vector<Eigen::Vector2d> missing_source{Eigen::Vector2d(-1000.0, -1000.0)};

    filter.set_particles_for_test({Particle{100.0, 100.0, 0.0, 1.0}});
    filter.update_weights(missing_source, target_map);
    EXPECT_TRUE(filter.needs_reinitialization());  // streak=1 >= reinit_consecutive_frames=1

    filter.set_particles_for_test({Particle{4.0, 0.0, 0.0, 1.0}});  // (1,0)->(5,0): 完全一致
    filter.update_weights(matching_source, target_map);
    EXPECT_FALSE(filter.needs_reinitialization());  // 良好なESS比でストリークがリセットされる
}
