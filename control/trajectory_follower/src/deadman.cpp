#include "trajectory_follower/deadman.hpp"

namespace trajectory_follower
{

bool is_stale(const rclcpp::Time & now, const rclcpp::Time & stamp, double timeout_s)
{
    return (now - stamp).seconds() > timeout_s;
}

}
