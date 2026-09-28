#pragma once

#include <speed_path_msgs/msg/speed_path.hpp>

namespace trajectory_follower
{

speed_path_msgs::msg::SpeedPathPoint preview_point(
    const speed_path_msgs::msg::SpeedPath & path_in_base,
    double v_meas,
    double preview_time);

}
