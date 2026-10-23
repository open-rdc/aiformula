#pragma once

#include <array>
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
void apply_stop(std::vector<double>& v, const std::vector<double>& s, double stop_s);
SPEED_PATH_PLANNER_PUBLIC
void backward_pass(std::vector<double>& v, const std::vector<double>& s, double a_lon);
SPEED_PATH_PLANNER_PUBLIC
void forward_pass(std::vector<double>& v, const std::vector<double>& s, double v_start, double a_lon);
SPEED_PATH_PLANNER_PUBLIC
std::vector<double> accelerations(const std::vector<double>& v, const std::vector<double>& s);
// 弧長 distance における速度を線形補間で返す（範囲外は端点の値）
SPEED_PATH_PLANNER_PUBLIC
double velocity_at_distance(
    const std::vector<double>& s, const std::vector<double>& velocity, double distance);
SPEED_PATH_PLANNER_PUBLIC
Profile plan(
    const std::vector<Point2D>& points, double v_start, double stop_s, const Limits& limits);

}
