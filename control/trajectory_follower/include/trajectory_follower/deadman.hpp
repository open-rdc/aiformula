#pragma once

#include <rclcpp/time.hpp>

namespace trajectory_follower
{

bool is_stale(const rclcpp::Time & now, const rclcpp::Time & stamp, double timeout_s);

}
