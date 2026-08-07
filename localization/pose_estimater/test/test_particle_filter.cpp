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
    config.odom_fw_dev_per_fw = 0.05;
    config.odom_fw_dev_per_rot = 0.0001;
    config.odom_rot_dev_per_fw = 0.001;
    config.odom_rot_dev_per_rot = 0.05;
    config.likelihood_dev = 0.3;
    config.likelihood_max_dist = 1.5;
    config.resample_ess_ratio_threshold = 0.5;
    config.reinit_residual_threshold = 1.0;
    config.reinit_consecutive_frames = 5;
    config.min_position_variance = 0.05;
    config.min_yaw_variance = 0.01;
    return config;
}

// 幅 2*half_width の直線車線を interval 間隔でサンプリングした地図点群。
std::vector<PfMapPoint> make_straight_lane_map(
    const double half_width, const double length, const double interval)
{
    std::vector<PfMapPoint> points;
    for (double s = -5.0; s <= length; s += interval) {
        points.push_back(PfMapPoint{Eigen::Vector2d(s, half_width)});
        points.push_back(PfMapPoint{Eigen::Vector2d(s, -half_width)});
    }
    return points;
}

// 真ポーズ(0,0,0)のbase_link系で見た前方 0.2..range m の両側車線観測。
std::vector<Eigen::Vector2d> make_lane_observation(
    const double half_width, const double range, const double interval)
{
    std::vector<Eigen::Vector2d> points;
    for (double s = 0.2; s <= range; s += interval) {
        points.emplace_back(s, half_width);
        points.emplace_back(s, -half_width);
    }
    return points;
}

double weight_of(const ParticleFilter& filter, const std::size_t index)
{
    return filter.particles()[index].weight;
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
    config.odom_fw_dev_per_fw = 0.0;
    config.odom_fw_dev_per_rot = 0.0;
    config.odom_rot_dev_per_fw = 0.0;
    config.odom_rot_dev_per_rot = 0.0;
    ParticleFilter filter(config, 3U);
    filter.set_particles_for_test({Particle{0.0, 0.0, 0.0, 1.0}});

    filter.predict(/*linear_velocity=*/2.0, /*yaw_rate=*/0.5, /*dt=*/0.1);

    const auto& particles = filter.particles();
    ASSERT_EQ(particles.size(), 1U);
    EXPECT_NEAR(particles[0].x, 0.2, 1e-9);
    EXPECT_NEAR(particles[0].y, 0.0, 1e-9);
    EXPECT_NEAR(particles[0].yaw, 0.05, 1e-9);
}

TEST(ParticleFilterTest, PredictFreezesParticlesWhenStationary)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 100U;
    ParticleFilter filter(config, 9U);
    std::vector<Particle> particles(100U, Particle{1.0, 2.0, 0.3, 0.01});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/0.0, /*yaw_rate=*/0.0, /*dt=*/0.1);

    for (const auto& particle : filter.particles()) {
        EXPECT_DOUBLE_EQ(particle.x, 1.0);
        EXPECT_DOUBLE_EQ(particle.y, 2.0);
        EXPECT_DOUBLE_EQ(particle.yaw, 0.3);
    }
}

TEST(ParticleFilterTest, PredictForwardNoiseSpreadsAlongTrackOnly)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 4000U;
    config.odom_fw_dev_per_fw = 0.1;
    config.odom_fw_dev_per_rot = 0.0;
    config.odom_rot_dev_per_fw = 0.0;
    config.odom_rot_dev_per_rot = 0.0;
    ParticleFilter filter(config, 11U);
    std::vector<Particle> particles(4000U, Particle{0.0, 0.0, 0.0, 1.0 / 4000.0});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/1.0, /*yaw_rate=*/0.0, /*dt=*/1.0);  // Δs=1

    double sum_sq = 0.0;
    for (const auto& particle : filter.particles()) {
        const double along = particle.x - 1.0;
        sum_sq += along * along;
        // 回転係数が0なら direction = yaw = 0 のまま。横方向は厳密にゼロ。
        EXPECT_DOUBLE_EQ(particle.y, 0.0);
        EXPECT_DOUBLE_EQ(particle.yaw, 0.0);
    }
    const double sample_std = std::sqrt(sum_sq / 4000.0);
    EXPECT_NEAR(sample_std, 0.1, 0.02);  // ff*sqrt(Δs) = 0.1
}

TEST(ParticleFilterTest, PredictRotationNoiseScalesWithRotation)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 4000U;
    config.odom_fw_dev_per_fw = 0.0;
    config.odom_fw_dev_per_rot = 0.0;
    config.odom_rot_dev_per_fw = 0.0;
    config.odom_rot_dev_per_rot = 0.2;
    ParticleFilter filter(config, 13U);
    std::vector<Particle> particles(4000U, Particle{0.0, 0.0, 0.0, 1.0 / 4000.0});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/0.0, /*yaw_rate=*/1.0, /*dt=*/0.25);  // Δθ=0.25

    double sum_sq = 0.0;
    for (const auto& particle : filter.particles()) {
        const double dyaw = particle.yaw - 0.25;
        sum_sq += dyaw * dyaw;
        // 前進係数が0かつ v=0 なら位置は動かない。
        EXPECT_DOUBLE_EQ(particle.x, 0.0);
        EXPECT_DOUBLE_EQ(particle.y, 0.0);
    }
    const double sample_std = std::sqrt(sum_sq / 4000.0);
    EXPECT_NEAR(sample_std, 0.1, 0.02);  // rr*sqrt(Δθ) = 0.2*0.5 = 0.1
}

TEST(ParticleFilterTest, PredictYawSpreadsWhileDrivingStraight)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 4000U;
    config.odom_fw_dev_per_fw = 0.0;
    config.odom_fw_dev_per_rot = 0.0;
    config.odom_rot_dev_per_fw = 0.05;
    config.odom_rot_dev_per_rot = 0.0;
    ParticleFilter filter(config, 17U);
    std::vector<Particle> particles(4000U, Particle{0.0, 0.0, 0.0, 1.0 / 4000.0});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/4.0, /*yaw_rate=*/0.0, /*dt=*/0.25);  // Δs=1

    double sum_sq = 0.0;
    for (const auto& particle : filter.particles()) {
        sum_sq += particle.yaw * particle.yaw;
    }
    const double sample_std = std::sqrt(sum_sq / 4000.0);
    EXPECT_NEAR(sample_std, 0.05, 0.01);  // rf*sqrt(Δs) = 0.05
}

TEST(ParticleFilterTest, PredictForwardNoiseFromRotation)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 4000U;
    config.odom_fw_dev_per_fw = 0.0;
    config.odom_fw_dev_per_rot = 0.1;
    config.odom_rot_dev_per_fw = 0.0;
    config.odom_rot_dev_per_rot = 0.0;
    ParticleFilter filter(config, 19U);
    std::vector<Particle> particles(4000U, Particle{0.0, 0.0, 0.0, 1.0 / 4000.0});
    filter.set_particles_for_test(particles);

    filter.predict(/*linear_velocity=*/0.0, /*yaw_rate=*/1.0, /*dt=*/1.0);  // Δθ=1

    double sum_sq = 0.0;
    for (const auto& particle : filter.particles()) {
        sum_sq += particle.x * particle.x;
        // 回転係数が0なら direction = yaw = 0。前進ノイズは x 軸方向のみ。
        EXPECT_DOUBLE_EQ(particle.y, 0.0);
        EXPECT_DOUBLE_EQ(particle.yaw, 1.0);
    }
    const double sample_std = std::sqrt(sum_sq / 4000.0);
    EXPECT_NEAR(sample_std, 0.1, 0.02);  // fr*sqrt(Δθ) = 0.1
}

TEST(ParticleFilterTest, PredictAccumulatedSpreadIsStepInvariant)
{
    const auto accumulated_std = [](const unsigned seed, const int steps) {
        ParticleFilterConfig config = make_default_config();
        config.num_particles = 4000U;
        config.odom_fw_dev_per_fw = 0.1;
        config.odom_fw_dev_per_rot = 0.0;
        config.odom_rot_dev_per_fw = 0.0;
        config.odom_rot_dev_per_rot = 0.0;
        ParticleFilter filter(config, seed);
        std::vector<Particle> particles(4000U, Particle{0.0, 0.0, 0.0, 1.0 / 4000.0});
        filter.set_particles_for_test(particles);

        const double dt = 1.0 / static_cast<double>(steps);
        for (int i = 0; i < steps; ++i) {
            filter.predict(/*linear_velocity=*/1.0, /*yaw_rate=*/0.0, dt);
        }
        double sum_sq = 0.0;
        for (const auto& particle : filter.particles()) {
            const double along = particle.x - 1.0;
            sum_sq += along * along;
        }
        return std::sqrt(sum_sq / 4000.0);
    };

    // 同じ1mの走行を1分割/10分割しても累積拡散は ff*sqrt(1m) = 0.1 で不変。
    EXPECT_NEAR(accumulated_std(23U, 1), 0.1, 0.02);
    EXPECT_NEAR(accumulated_std(29U, 10), 0.1, 0.02);
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
    config.likelihood_dev = 0.3;
    config.likelihood_max_dist = 2.0;
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
    // 対応点が無い観測は最大対応距離ぶんの罰を受けるだけで、重みは0にならない。
    EXPECT_GT(particles[2].weight, 0.0);

    double weight_sum = 0.0;
    for (const auto& particle : particles) {
        weight_sum += particle.weight;
    }
    EXPECT_NEAR(weight_sum, 1.0, 1e-9);
}

TEST(ParticleFilterTest, UpdateWeightsPrefersTruePoseOverLaneWidthAlias)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    ParticleFilter filter(config, 5U);

    const double half_width = 1.5;
    const PfTargetMap target_map(make_straight_lane_map(half_width, 60.0, 0.25));
    const auto observation = make_lane_observation(half_width, 30.0, 0.25);

    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 0.5},                // 真ポーズ
        Particle{0.0, 2.0 * half_width, 0.0, 0.5},   // 車線幅ぶん横にずれたエイリアス
    });

    filter.update_weights(observation, target_map);

    EXPECT_GT(weight_of(filter, 0), weight_of(filter, 1));
}

TEST(ParticleFilterTest, UpdateWeightsKeepsWeightPositiveWithoutCorrespondence)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    ParticleFilter filter(config, 5U);

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> observation{Eigen::Vector2d(1.0, 0.0)};

    filter.set_particles_for_test({
        Particle{4.0, 0.0, 0.0, 0.5},       // (1,0)->(5,0): 完全一致
        Particle{100.0, 100.0, 0.0, 0.5},   // 対応点なし
    });

    filter.update_weights(observation, target_map);

    EXPECT_GT(weight_of(filter, 0), weight_of(filter, 1));
    EXPECT_GT(weight_of(filter, 1), 0.0);
}

TEST(ParticleFilterTest, UpdateWeightsSharpnessIsIndependentOfPointCount)
{
    const auto weight_ratio_for = [](const std::size_t point_count) {
        ParticleFilterConfig config = make_default_config();
        config.num_particles = 2U;
        ParticleFilter filter(config, 5U);

        std::vector<PfMapPoint> map_points;
        for (std::size_t i = 0U; i < 400U; ++i) {
            map_points.push_back(
                PfMapPoint{Eigen::Vector2d(0.25 * static_cast<double>(i), 0.0)});
        }
        const PfTargetMap target_map(std::move(map_points));

        std::vector<Eigen::Vector2d> observation;
        for (std::size_t i = 0U; i < point_count; ++i) {
            observation.emplace_back(1.0 + 0.25 * static_cast<double>(i), 0.0);
        }

        filter.set_particles_for_test({
            Particle{0.0, 0.0, 0.0, 0.5},   // 全点で残差0
            Particle{0.0, 0.3, 0.0, 0.5},   // 全点で残差0.3m
        });
        filter.update_weights(observation, target_map);
        return weight_of(filter, 1) / weight_of(filter, 0);
    };

    // 残差0.3m / likelihood_dev 0.3m の重み比は exp(-0.5)。
    // 観測点数で正規化されていれば点数に依らずこの値になる。
    // 累積のままだと exp(-0.5*N) となり点数の指数で変わってしまう。
    const double expected = std::exp(-0.5);
    EXPECT_NEAR(weight_ratio_for(20U), expected, 0.02);
    EXPECT_NEAR(weight_ratio_for(40U), expected, 0.02);
}

TEST(ParticleFilterTest, UpdateWeightsDiscriminatesWithManyObservationPoints)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 3U;
    ParticleFilter filter(config, 5U);

    const double half_width = 1.5;
    const PfTargetMap target_map(make_straight_lane_map(half_width, 300.0, 0.25));
    const auto observation = make_lane_observation(half_width, 30.0, 0.03);
    ASSERT_GE(observation.size(), 1900U);

    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 1.0 / 3.0},
        Particle{0.0, 0.2, 0.0, 1.0 / 3.0},
        Particle{0.0, 0.8, 0.0, 1.0 / 3.0},
    });

    filter.update_weights(observation, target_map);

    EXPECT_GT(weight_of(filter, 0), weight_of(filter, 1));
    EXPECT_GT(weight_of(filter, 1), weight_of(filter, 2));
    EXPECT_GT(weight_of(filter, 2), 0.0);
}

TEST(ParticleFilterTest, NeedsReinitializationWhenBestResidualStaysLarge)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    config.likelihood_max_dist = 1.5;
    config.reinit_residual_threshold = 0.5;
    config.reinit_consecutive_frames = 3;
    ParticleFilter filter(config, 5U);

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> observation{Eigen::Vector2d(1.0, 0.0)};

    // どちらも残差1.2mで対等。重みは均一になるためESS比は1.0だが、
    // 地図には全く合っていないので見失いとして扱う必要がある。
    filter.set_particles_for_test({
        Particle{4.0, 1.2, 0.0, 0.5},
        Particle{4.0, -1.2, 0.0, 0.5},
    });

    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(observation, target_map);
    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(observation, target_map);
    EXPECT_FALSE(filter.needs_reinitialization());
    filter.update_weights(observation, target_map);
    EXPECT_TRUE(filter.needs_reinitialization());
}

TEST(ParticleFilterTest, SmallResidualResetsReinitStreak)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    config.likelihood_max_dist = 1.5;
    config.reinit_residual_threshold = 0.5;
    config.reinit_consecutive_frames = 1;
    ParticleFilter filter(config, 5U);

    const PfTargetMap target_map(std::vector<PfMapPoint>{PfMapPoint{Eigen::Vector2d(5.0, 0.0)}});
    const std::vector<Eigen::Vector2d> observation{Eigen::Vector2d(1.0, 0.0)};

    filter.set_particles_for_test({
        Particle{4.0, 1.2, 0.0, 0.5},
        Particle{4.0, -1.2, 0.0, 0.5},
    });
    filter.update_weights(observation, target_map);
    EXPECT_TRUE(filter.needs_reinitialization());

    filter.set_particles_for_test({
        Particle{4.0, 0.0, 0.0, 0.5},
        Particle{4.0, 0.05, 0.0, 0.5},
    });
    filter.update_weights(observation, target_map);
    EXPECT_FALSE(filter.needs_reinitialization());
}

TEST(ParticleFilterTest, EstimateComputesWeightedMeanAndCovariance)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    ParticleFilter filter(config, 1U);
    filter.set_particles_for_test({
        Particle{0.0, 0.0, 0.0, 0.5},
        Particle{2.0, 0.0, 0.0, 0.5},
    });

    const auto estimate = filter.estimate();

    EXPECT_NEAR(estimate.x, 1.0, 1e-9);
    EXPECT_NEAR(estimate.y, 0.0, 1e-9);
    EXPECT_NEAR(estimate.yaw, 0.0, 1e-9);
    EXPECT_NEAR(estimate.position_covariance(0, 0), 1.0, 1e-9);
}

TEST(ParticleFilterTest, EstimateClampsCovarianceToFloor)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 3U;
    config.min_position_variance = 0.05;
    config.min_yaw_variance = 0.01;
    ParticleFilter filter(config, 1U);

    // リサンプルで退化し全粒子が同一ポーズに潰れた状態。粒子分散は0になるが、
    // 実際の推定誤差が0でないことを下限で表明する必要がある。
    filter.set_particles_for_test({
        Particle{1.0, 2.0, 0.3, 1.0 / 3.0},
        Particle{1.0, 2.0, 0.3, 1.0 / 3.0},
        Particle{1.0, 2.0, 0.3, 1.0 / 3.0},
    });

    const auto estimate = filter.estimate();

    EXPECT_NEAR(estimate.x, 1.0, 1e-9);
    EXPECT_NEAR(estimate.y, 2.0, 1e-9);
    EXPECT_GE(estimate.position_covariance(0, 0), 0.05);
    EXPECT_GE(estimate.position_covariance(1, 1), 0.05);
    EXPECT_GE(estimate.yaw_variance, 0.01);
}

TEST(ParticleFilterTest, EstimateYawHandlesWrapAroundNearPi)
{
    ParticleFilterConfig config = make_default_config();
    config.num_particles = 2U;
    ParticleFilter filter(config, 1U);
    const double near_pi = M_PI - 0.01;
    filter.set_particles_for_test({
        Particle{0.0, 0.0, near_pi, 0.5},
        Particle{0.0, 0.0, -near_pi, 0.5},
    });

    const auto estimate = filter.estimate();

    EXPECT_NEAR(std::abs(estimate.yaw), M_PI, 1e-2);
}
