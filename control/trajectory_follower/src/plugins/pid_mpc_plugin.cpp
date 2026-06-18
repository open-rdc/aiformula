#include "trajectory_follower/plugins/pid_mpc_plugin.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace trajectory_follower
{

namespace
{
constexpr double DEG2RAD = 0.017453292519943295;
constexpr double EPSILON = 1.0e-9;

double menger_curvature(
    const std::array<double, 2> & p0,
    const std::array<double, 2> & p1,
    const std::array<double, 2> & p2)
{
    const double area2 =
        (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    const double a = std::hypot(p1[0] - p0[0], p1[1] - p0[1]);
    const double b = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const double c = std::hypot(p2[0] - p0[0], p2[1] - p0[1]);
    const double denom = a * b * c;
    if (denom < EPSILON) {
        return 0.0;
    }
    return 2.0 * area2 / denom;
}
}  // namespace

void PidMpcPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    auto get_d = [&](const std::string & name, double def) {
        return params->has_parameter(name) ? params->get_parameter(name).get_value<double>() : def;
    };
    auto get_i = [&](const std::string & name, int def) {
        return params->has_parameter(name) ? params->get_parameter(name).get_value<int>() : def;
    };

    const double dt = static_cast<double>(get_i("control_period_ms", 50)) / 1000.0;

    LongitudinalParams lon;
    lon.v_max = get_d("linear_max.vel", 2.0);
    lon.a_lat_max = get_d("mpc.longitudinal.a_lat_max", 2.0);
    lon.a_max = get_d("mpc.longitudinal.a_max", 1.0);
    lon.a_min = get_d("mpc.longitudinal.a_min", -2.0);
    lon.jerk_max = get_d("mpc.longitudinal.jerk_max", 2.0);
    lon.kp = get_d("mpc.longitudinal.kp", 0.8);
    lon.ki = get_d("mpc.longitudinal.ki", 0.1);
    lon.kd = get_d("mpc.longitudinal.kd", 0.0);
    lon.lpf_vel_error_gain = get_d("mpc.longitudinal.lpf_vel_error_gain", 0.9);
    lon.stop_distance = get_d("mpc.longitudinal.stop_distance", 0.5);
    lon.dt = dt;
    longitudinal_.configure(lon);

    LateralMpcParams lat;
    lat.wheelbase = get_d("wheelbase", 0.8);
    lat.steer_tau = get_d("mpc.lateral.steer_tau", 0.3);
    lat.steer_limit = get_d("steering_max.pos", 15.0) * DEG2RAD;
    lat.weight_lat_error = get_d("mpc.lateral.weight_lat_error", 1.0);
    lat.weight_heading_error = get_d("mpc.lateral.weight_heading_error", 1.0);
    lat.weight_steering_input = get_d("mpc.lateral.weight_steering_input", 0.5);
    lat.weight_steer_rate = get_d("mpc.lateral.weight_steer_rate", 5.0);
    lat.weight_terminal_lat_error = get_d("mpc.lateral.weight_terminal_lat_error", 1.0);
    lat.weight_terminal_heading_error = get_d("mpc.lateral.weight_terminal_heading_error", 1.0);
    lat.horizon = get_i("mpc.lateral.horizon", 20);
    lat.prediction_dt = get_d("mpc.lateral.prediction_dt", 0.1);
    lat.min_predict_speed = get_d("mpc.lateral.min_predict_speed", 0.5);
    lateral_.configure(lat);

    if (lon.v_max <= 0.0 || lat.wheelbase <= 0.0 || lat.steer_limit <= 0.0) {
        throw std::invalid_argument("PidMpcPlugin: control parameters are invalid");
    }

    RCLCPP_INFO(
        logger_,
        "PidMpcPlugin 初期化: 縦PID(kp=%.2f ki=%.2f kd=%.2f) + 横MPC(N=%d dt=%.2f)",
        lon.kp, lon.ki, lon.kd, lat.horizon, lat.prediction_dt);
}

std::optional<steered_drive_msg::msg::SteeredDrive> PidMpcPlugin::computeCommand(
    const nav_msgs::msg::Path & path_in_base,
    double current_velocity,
    geometry_msgs::msg::PoseStamped & target_pose_out)
{
    const auto & poses = path_in_base.poses;
    const int n = static_cast<int>(poses.size());
    if (n < 3) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "経路点が不足 (3点未満)");
        return std::nullopt;
    }

    std::vector<std::array<double, 2>> path_xy;
    path_xy.reserve(n);
    for (const auto & pose : poses) {
        path_xy.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    // 累積距離と最近傍点
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i) {
        arc[i] = arc[i - 1] + std::hypot(path_xy[i][0] - path_xy[i - 1][0],
                                         path_xy[i][1] - path_xy[i - 1][1]);
    }
    int nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        const double d = path_xy[i][0] * path_xy[i][0] + path_xy[i][1] * path_xy[i][1];
        if (d < best) {
            best = d;
            nearest = i;
        }
    }

    const double dist_to_end = arc[n - 1] - arc[nearest];

    // 前方ウィンドウの最大曲率で参照速度を制限 (カーブ手前で減速)
    double curvature = 0.0;
    const double window = 3.0;  // [m]
    for (int i = std::max(1, nearest); i < n - 1; ++i) {
        if (arc[i] - arc[nearest] > window) break;
        curvature = std::max(curvature, std::abs(menger_curvature(path_xy[i - 1], path_xy[i], path_xy[i + 1])));
    }

    const double v_ref = longitudinal_.referenceSpeed(curvature, dist_to_end);
    const double v_cmd = longitudinal_.update(v_ref, current_velocity);
    const double steer = lateral_.computeSteering(path_xy, v_ref);

    // 可視化用の目標点 (前方 lookahead 上の経路点)
    int target_idx = nearest;
    for (int i = nearest; i < n; ++i) {
        if (arc[i] - arc[nearest] >= 2.0) {
            target_idx = i;
            break;
        }
        target_idx = i;
    }
    target_pose_out.pose.position.x = path_xy[target_idx][0];
    target_pose_out.pose.position.y = path_xy[target_idx][1];
    target_pose_out.pose.position.z = 0.0;
    target_pose_out.pose.orientation.w = 1.0;

    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = v_cmd;
    command.steering_angle = steer;
    return command;
}

}  // namespace trajectory_follower

PLUGINLIB_EXPORT_CLASS(trajectory_follower::PidMpcPlugin, trajectory_follower::ControllerPlugin)
