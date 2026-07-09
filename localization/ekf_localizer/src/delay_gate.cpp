#include "ekf_localizer/delay_gate.hpp"

#include <algorithm>

namespace ekf_localizer
{

DelayGateResult check_delay_gate(
    const rclcpp::Time& now,
    const rclcpp::Time& stamp,
    const double additional_delay_s,
    const double max_delay_s)
{
    const double raw_delay_time_s = (now - stamp).seconds() + additional_delay_s;
    const double delay_time_s = std::max(raw_delay_time_s, 0.0);
    return DelayGateResult{delay_time_s, delay_time_s <= max_delay_s};
}

}
