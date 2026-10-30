#pragma once

#include <array>
#include <optional>
#include <vector>
#include "speed_path_planner/visibility_control.h"

namespace speed_path_planner::speed_profile
{

using Point2D = std::array<double, 2>;

struct Limits
{
    double v_max;
    double a_lon;
    double a_lat_max;
    double stop_decel;
    double stop_decel_max;
};

struct Profile
{
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> curvature;
};

SPEED_PATH_PLANNER_PUBLIC
std::vector<double> arc_lengths(const std::vector<Point2D>& points);
SPEED_PATH_PLANNER_PUBLIC
std::vector<double> curvatures(const std::vector<Point2D>& points);
SPEED_PATH_PLANNER_PUBLIC
std::vector<double> lateral_limits(const std::vector<double>& curvature, const Limits& limits);
SPEED_PATH_PLANNER_PUBLIC
void apply_stop(std::vector<double>& v, const std::vector<double>& s, double stop_s, double decel);
SPEED_PATH_PLANNER_PUBLIC
void backward_pass(std::vector<double>& v, const std::vector<double>& s, double a_lon);
SPEED_PATH_PLANNER_PUBLIC
void forward_pass(std::vector<double>& v, const std::vector<double>& s, double v_start, double a_lon);
SPEED_PATH_PLANNER_PUBLIC
std::vector<double> accelerations(const std::vector<double>& v, const std::vector<double>& s);
SPEED_PATH_PLANNER_PUBLIC
Profile plan(
    const std::vector<Point2D>& points, double v_meas, double stop_s, const Limits& limits);
// 外から一時的に与えられた速度上限 (e2e_planner の /planning/speed_limit) を v_max に重ねる。
// 上限が無いか timeout_s より古ければ v_max のまま。v_max を上げることはしない
SPEED_PATH_PLANNER_PUBLIC
double effective_v_max(double v_max, std::optional<double> limit, double limit_age_s, double timeout_s);

}
