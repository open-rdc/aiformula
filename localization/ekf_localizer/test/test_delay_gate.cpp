#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/delay_gate.hpp"

using ekf_localizer::check_delay_gate;

TEST(DelayGate, PassesWhenWithinMaxDelay)
{
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const rclcpp::Time now(1, 300000000, RCL_ROS_TIME);  // +0.3s

    const auto result = check_delay_gate(now, stamp, 0.0, 0.5);
    EXPECT_TRUE(result.passed);
    EXPECT_NEAR(result.delay_time_s, 0.3, 1e-9);
}

TEST(DelayGate, RejectsWhenExceedingMaxDelay)
{
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const rclcpp::Time now(2, 0, RCL_ROS_TIME);  // +1.0s

    const auto result = check_delay_gate(now, stamp, 0.0, 0.5);
    EXPECT_FALSE(result.passed);
    EXPECT_NEAR(result.delay_time_s, 1.0, 1e-9);
}

TEST(DelayGate, AdditionalDelayIsAddedToMeasuredDelay)
{
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const rclcpp::Time now(1, 200000000, RCL_ROS_TIME);  // +0.2s

    const auto result = check_delay_gate(now, stamp, 0.2, 0.5);
    EXPECT_NEAR(result.delay_time_s, 0.4, 1e-9);
    EXPECT_TRUE(result.passed);
}

TEST(DelayGate, ClampsNegativeDelayToZero)
{
    const rclcpp::Time stamp(2, 0, RCL_ROS_TIME);
    const rclcpp::Time now(1, 0, RCL_ROS_TIME);  // stamp is in the future relative to now

    const auto result = check_delay_gate(now, stamp, 0.0, 0.5);
    EXPECT_TRUE(result.passed);
    EXPECT_NEAR(result.delay_time_s, 0.0, 1e-9);
}

TEST(DelayGate, BoundaryEqualToMaxDelayPasses)
{
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const rclcpp::Time now(1, 500000000, RCL_ROS_TIME);  // exactly +0.5s

    const auto result = check_delay_gate(now, stamp, 0.0, 0.5);
    EXPECT_TRUE(result.passed);
}
