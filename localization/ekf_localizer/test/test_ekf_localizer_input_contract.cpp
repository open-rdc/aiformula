#include <gtest/gtest.h>

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
    config.process_position_variance = 0.01;
    config.process_yaw_variance = 0.01;
    config.process_velocity_variance = 0.01;
    config.process_yaw_rate_variance = 0.01;
    config.position_gate_dist = 3.0;
    config.yaw_gate_dist = 3.0;
    config.min_position_variance = 0.0;
    config.min_yaw_variance = 0.0;
    config.position_gate_max_reject_duration_s = 1.0;
    config.yaw_gate_max_reject_duration_s = 1.0;
    return config;
}

}

TEST(EkfLocalizerInputContract, PredictIgnoresBackwardsTimestamp)
{
    EkfLocalizer ekf(make_config());
    const rclcpp::Time init_stamp(10, 0, RCL_ROS_TIME);
    ekf.initialize(1.0, 2.0, 0.5, init_stamp);

    const rclcpp::Time backwards_stamp(5, 0, RCL_ROS_TIME);
    EXPECT_NO_THROW(ekf.predict(1.0, 0.1, backwards_stamp));

    const auto pose = ekf.make_pose("map");
    EXPECT_DOUBLE_EQ(pose.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, 2.0);
}
