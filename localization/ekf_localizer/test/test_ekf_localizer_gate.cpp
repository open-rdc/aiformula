#include <gtest/gtest.h>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/ekf_localizer.hpp"

using ekf_localizer::EkfLocalizer;
using ekf_localizer::EkfLocalizerConfig;

namespace
{

EkfLocalizerConfig make_config()
{
    EkfLocalizerConfig config;
    config.initial_position_variance = 1.0;
    config.initial_yaw_variance = 0.3;
    config.process_position_variance = 0.1;
    config.process_yaw_variance = 0.05;
    config.process_velocity_variance = 0.5;
    config.process_yaw_rate_variance = 0.2;
    config.position_gate_dist = 3.0;
    config.yaw_gate_dist = 3.0;
    return config;
}

}

TEST(EkfLocalizerGate, AcceptsCloseUpdateAndRejectsPositionOutlier)
{
    EkfLocalizer ekf(make_config());
    const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, t0);

    const rclcpp::Time t1 = t0 + rclcpp::Duration::from_seconds(0.1);
    const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.1;
    EXPECT_TRUE(ekf.update_position(0.05, 0.02, cov, t1));

    const rclcpp::Time t2 = t1 + rclcpp::Duration::from_seconds(0.1);
    EXPECT_FALSE(ekf.update_position(50.0, 50.0, cov, t2));
}

TEST(EkfLocalizerGate, AcceptsCloseYawAndRejectsYawOutlier)
{
    EkfLocalizer ekf(make_config());
    const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
    ekf.initialize(0.0, 0.0, 0.0, t0);

    const rclcpp::Time t1 = t0 + rclcpp::Duration::from_seconds(0.1);
    EXPECT_TRUE(ekf.update_yaw(0.02, 0.05, t1));

    const rclcpp::Time t2 = t1 + rclcpp::Duration::from_seconds(0.1);
    EXPECT_FALSE(ekf.update_yaw(2.5, 0.05, t2));
}
