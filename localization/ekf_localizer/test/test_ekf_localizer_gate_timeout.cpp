#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/ekf_localizer.hpp"

using ekf_localizer::EkfLocalizer;
using ekf_localizer::EkfLocalizerConfig;

namespace
{

EkfLocalizerConfig make_config(
    const double position_gate_max_reject_duration_s,
    const double yaw_gate_max_reject_duration_s)
{
    EkfLocalizerConfig config;
    config.initial_position_variance = 1.0;
    config.initial_yaw_variance = 0.3;
    config.process_position_variance = 0.0;
    config.process_yaw_variance = 0.0;
    config.process_velocity_variance = 0.0;
    config.process_yaw_rate_variance = 0.0;
    config.position_gate_dist = 3.0;
    config.yaw_gate_dist = 3.0;
    config.min_position_variance = 0.0;
    config.min_yaw_variance = 0.0;
    config.position_gate_max_reject_duration_s = position_gate_max_reject_duration_s;
    config.yaw_gate_max_reject_duration_s = yaw_gate_max_reject_duration_s;
    return config;
}

}

TEST(EkfLocalizerGateTimeout, ForcesPositionAcceptanceOnceRejectedElapsedTimeExceedsTimeout)
{
    // 実車ログ再現: 自律走行開始直後の急加速でEKFが静止したまま積分を続け、
    // 実位置と約20m以上乖離した状態のままpf_pose(実位置)を送り続けても
    // mahalanobisゲートが弾き続ける「二重ロック」を想定したテスト。
    // position_gate_max_reject_duration_sを超えた経過時間以内には強制受理されるべき。
    constexpr double max_reject_duration_s = 0.3;
    constexpr double dt = 0.02;
    EkfLocalizer ekf(make_config(max_reject_duration_s, 1.0));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    const Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity() * 0.5;

    // has_last_position_stamp_をセットしておく（初回のupdate_position呼び出しは
    // 前回タイムスタンプがないためdt=0扱いとなり経過時間計測がずれるため、
    // 一致する観測を1回受理させてから計測を始める）。
    stamp = stamp + rclcpp::Duration::from_seconds(dt);
    ekf.predict(0.0, 0.0, stamp);
    ASSERT_TRUE(ekf.update_position(0.0, 0.0, measurement_covariance, stamp));

    double elapsed = 0.0;
    bool accepted = false;
    for (int i = 0; i < 30; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(dt);
        ekf.predict(0.0, 0.0, stamp);
        const bool passed = ekf.update_position(15.0, -30.0, measurement_covariance, stamp);
        elapsed += dt;
        if (passed) {
            accepted = true;
            break;
        }
    }

    ASSERT_TRUE(accepted);
    EXPECT_LE(elapsed, max_reject_duration_s + dt + 1e-6);
}

TEST(EkfLocalizerGateTimeout, DoesNotForcePositionAcceptBeforeTimeoutElapsed)
{
    // タイムアウト到達前の単発の大外れ値は、今回の変更後もこれまで通り
    // mahalanobisゲートで棄却されなければならない（外れ値除去能力の回帰テスト）。
    EkfLocalizer ekf(make_config(0.3, 0.3));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    stamp = stamp + rclcpp::Duration::from_seconds(0.02);
    ekf.predict(0.0, 0.0, stamp);
    const Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity() * 0.5;
    EXPECT_FALSE(ekf.update_position(15.0, -30.0, measurement_covariance, stamp));
}

TEST(EkfLocalizerGateTimeout, ForcesYawAcceptanceOnceRejectedElapsedTimeExceedsTimeout)
{
    // yawゲートについても位置ゲートと同様、連続棄却の累積経過時間が
    // yaw_gate_max_reject_duration_sを超えたら強制的に受理されるべき。
    constexpr double max_reject_duration_s = 0.3;
    constexpr double dt = 0.02;
    EkfLocalizer ekf(make_config(1.0, max_reject_duration_s));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    constexpr double yaw_measurement_variance = 0.05;

    // has_last_yaw_stamp_をセットしておく（初回のupdate_yaw呼び出しは
    // 前回タイムスタンプがないためdt=0扱いとなり経過時間計測がずれるため、
    // 一致する観測を1回受理させてから計測を始める）。
    stamp = stamp + rclcpp::Duration::from_seconds(dt);
    ekf.predict(0.0, 0.0, stamp);
    ASSERT_TRUE(ekf.update_yaw(0.0, yaw_measurement_variance, stamp));

    double elapsed = 0.0;
    bool accepted = false;
    for (int i = 0; i < 30; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(dt);
        ekf.predict(0.0, 0.0, stamp);
        const bool passed = ekf.update_yaw(M_PI, yaw_measurement_variance, stamp);
        elapsed += dt;
        if (passed) {
            accepted = true;
            break;
        }
    }

    ASSERT_TRUE(accepted);
    EXPECT_LE(elapsed, max_reject_duration_s + dt + 1e-6);
}

TEST(EkfLocalizerGateTimeout, DoesNotForceYawAcceptBeforeTimeoutElapsed)
{
    // タイムアウト到達前の単発の大外れ値は、今回の変更後もこれまで通り
    // mahalanobisゲートで棄却されなければならない（外れ値除去能力の回帰テスト）。
    EkfLocalizer ekf(make_config(0.3, 0.3));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    stamp = stamp + rclcpp::Duration::from_seconds(0.02);
    ekf.predict(0.0, 0.0, stamp);
    EXPECT_FALSE(ekf.update_yaw(M_PI, 0.05, stamp));
}
