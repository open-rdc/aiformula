#include "frenet_planner/risk_calculator.hpp"
#include "frenet_planner/frenet_planner.hpp"
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace frenet_planner {

RiskCalculator::RiskCalculator()
    : k_jerk_(0.0),
      k_time_(0.0),
      k_d_(0.0),
      k_s_dot_(0.0),
      target_speed_(0.0) {
}

void RiskCalculator::set_parameters(
    double k_jerk,
    double k_time,
    double k_d,
    double k_s_dot,
    double target_speed
) {
    k_jerk_ = k_jerk;
    k_time_ = k_time;
    k_d_ = k_d;
    k_s_dot_ = k_s_dot;
    target_speed_ = target_speed;
}

double RiskCalculator::calculate_cost(
    const FrenetTrajectory& trajectory,
    const std::vector<Obstacle>& obstacles
) {
    (void)obstacles;
    if (trajectory.points.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("frenet_planner"),
            "calculate_cost: Empty trajectory!");
        return std::numeric_limits<double>::max();
    }

    if (trajectory.t.empty() || trajectory.s.empty() || trajectory.d.empty() || trajectory.s_dot.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("frenet_planner"),
            "calculate_cost: Missing Frenet samples!");
        return std::numeric_limits<double>::max();
    }

    double lateral_jerk_sum = 0.0;
    for (const auto& jerk : trajectory.d_dddot) {
        lateral_jerk_sum += jerk * jerk;
    }

    double longitudinal_jerk_sum = 0.0;
    for (const auto& jerk : trajectory.s_dddot) {
        longitudinal_jerk_sum += jerk * jerk;
    }

    const double T = trajectory.t.back();
    const double d_end = trajectory.d.back();
    const double v_end = trajectory.s_dot.back();
    const double ds = target_speed_ - v_end;

    const double cd = k_jerk_ * lateral_jerk_sum + k_time_ * T + k_d_ * d_end * d_end;
    const double cv = k_jerk_ * longitudinal_jerk_sum + k_time_ * T + k_s_dot_ * ds * ds;

    return k_lateral_ * cd + k_longitudinal_ * cv;
}

}  // namespace frenet_planner
