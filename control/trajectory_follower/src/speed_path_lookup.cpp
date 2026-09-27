#include "trajectory_follower/speed_path_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_follower
{

speed_path_msgs::msg::SpeedPathPoint preview_point(
    const speed_path_msgs::msg::SpeedPath & path_in_base,
    double v_meas,
    double preview_time)
{
    const auto & points = path_in_base.points;
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto & p = points[i].pose.position;
        const double d = p.x * p.x + p.y * p.y;
        if (d < best) {
            best = d;
            nearest = i;
        }
    }

    const double target = std::max(std::max(v_meas, 0.0) * preview_time, 0.5);
    double s = 0.0;
    for (std::size_t i = nearest + 1; i < points.size(); ++i) {
        const auto & a = points[i - 1];
        const auto & b = points[i];
        const double ds = std::hypot(
            b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
        if (s + ds >= target) {
            const double r = (target - s) / ds;
            speed_path_msgs::msg::SpeedPathPoint out = b;
            out.linear_velocity = a.linear_velocity + r * (b.linear_velocity - a.linear_velocity);
            out.linear_acceleration =
                a.linear_acceleration + r * (b.linear_acceleration - a.linear_acceleration);
            out.curvature = a.curvature + r * (b.curvature - a.curvature);
            return out;
        }
        s += ds;
    }
    return points.back();
}

}
