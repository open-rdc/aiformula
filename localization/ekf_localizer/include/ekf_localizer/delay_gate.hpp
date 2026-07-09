#pragma once

#include <rclcpp/time.hpp>

namespace ekf_localizer
{

struct DelayGateResult
{
    double delay_time_s;
    bool passed;
};

DelayGateResult check_delay_gate(
    const rclcpp::Time& now,
    const rclcpp::Time& stamp,
    double additional_delay_s,
    double max_delay_s);

}
