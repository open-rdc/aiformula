#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/ekf_localizer.hpp"

using ekf_localizer::EkfLocalizer;
using ekf_localizer::EkfLocalizerConfig;

namespace
{

EkfLocalizerConfig make_config(
    const double min_position_variance, const double min_yaw_variance)
{
    EkfLocalizerConfig config;
    config.initial_position_variance = 1.0;
    config.initial_yaw_variance = 1.0;
    config.process_position_variance = 0.0;
    config.process_yaw_variance = 0.0;
    config.process_velocity_variance = 0.0;
    config.process_yaw_rate_variance = 0.0;
    config.position_gate_dist = 3.0;
    config.yaw_gate_dist = 3.0;
    config.min_position_variance = min_position_variance;
    config.min_yaw_variance = min_yaw_variance;
    config.position_gate_max_reject_duration_s = 1.0;
    config.yaw_gate_max_reject_duration_s = 1.0;
    return config;
}

}

TEST(EkfLocalizerCovarianceFloor, PositionVarianceNeverShrinksBelowFloor)
{
    // 停止継続時など同じ位置に一致し続けるPF観測が繰り返しacceptされると
    // 位置共分散が際限なく収縮しうる（実走行ログで確認されたゲートロックの回帰テスト）。
    constexpr double min_position_variance = 1.0;
    EkfLocalizer ekf(make_config(min_position_variance, 0.0));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    const Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity() * 0.001;
    for (int i = 0; i < 2000; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(0.02);
        ekf.predict(0.0, 0.0, stamp);
        ASSERT_TRUE(ekf.update_position(0.0, 0.0, measurement_covariance, stamp));
    }

    const auto pose = ekf.make_pose("map");
    EXPECT_GE(pose.pose.covariance[0], min_position_variance - 1e-9);
    EXPECT_GE(pose.pose.covariance[7], min_position_variance - 1e-9);
}

TEST(EkfLocalizerCovarianceFloor, RecoversQuicklyAfterStationaryStreakWhenPositionActuallyMoves)
{
    // フロアがあれば、長時間の一致観測ストリークの後でも、実際の移動に対応する
    // 正しい観測をわずかな棄却回数で速やかに受理できるはず
    // （実走行での「自律再開直後、数秒〜十数秒pf_poseが棄却され続ける」不具合の再現・回帰テスト）。
    constexpr double min_position_variance = 1.0;
    EkfLocalizer ekf(make_config(min_position_variance, 0.0));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    const Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity() * 0.001;
    for (int i = 0; i < 2000; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(0.02);
        ekf.predict(0.0, 0.0, stamp);
        ASSERT_TRUE(ekf.update_position(0.0, 0.0, measurement_covariance, stamp));
    }

    const Eigen::Matrix2d moved_measurement_covariance = Eigen::Matrix2d::Identity() * 0.05;
    int accept_step = -1;
    for (int i = 0; i < 10; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(0.02);
        ekf.predict(0.0, 0.0, stamp);
        if (ekf.update_position(1.0, 0.0, moved_measurement_covariance, stamp)) {
            accept_step = i;
            break;
        }
    }

    EXPECT_NE(accept_step, -1) << "EKF did not recover after a long stationary streak";
    EXPECT_LE(accept_step, 1)
        << "position variance floor should allow recovery within a step or two, not hundreds";
}

TEST(EkfLocalizerCovarianceFloor, YawVarianceNeverShrinksBelowFloor)
{
    constexpr double min_yaw_variance = 0.1;
    EkfLocalizer ekf(make_config(0.0, min_yaw_variance));

    rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, stamp);

    for (int i = 0; i < 2000; ++i) {
        stamp = stamp + rclcpp::Duration::from_seconds(0.02);
        ekf.predict(0.0, 0.0, stamp);
        ASSERT_TRUE(ekf.update_yaw(0.0, 0.001, stamp));
    }

    const auto pose = ekf.make_pose("map");
    EXPECT_GE(pose.pose.covariance[35], min_yaw_variance - 1e-9);
}
