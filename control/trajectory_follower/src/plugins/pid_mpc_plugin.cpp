#include "trajectory_follower/plugins/pid_mpc_plugin.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "trajectory_follower/mpc/speed_limit.hpp"
#include "utilities/utils.hpp"

namespace trajectory_follower
{

void PidMpcPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    const double dt =
        static_cast<double>(params->get_parameter("interval_ms").as_int()) / 1000.0;

    LongitudinalParams lon;
    lon.v_max = params->get_parameter("linear_max.vel").as_double();
    lon.a_lat_max = params->get_parameter("a_lat_max").as_double();
    lon.a_max = params->get_parameter("mpc.longitudinal.a_max").as_double();
    lon.a_min = params->get_parameter("a_min").as_double();
    lon.jerk_max = params->get_parameter("mpc.longitudinal.jerk_max").as_double();
    lon.kp = params->get_parameter("mpc.longitudinal.kp").as_double();
    lon.ki = params->get_parameter("mpc.longitudinal.ki").as_double();
    lon.kd = params->get_parameter("mpc.longitudinal.kd").as_double();
    lon.lpf_vel_error_gain = params->get_parameter("mpc.longitudinal.lpf_vel_error_gain").as_double();
    lon.dt = dt;
    longitudinal_.configure(lon);

    LateralMpcParams lat;
    lat.wheelbase = params->get_parameter("wheelbase").as_double();
    lat.steer_tau = params->get_parameter("mpc.lateral.steer_tau").as_double();
    lat.steer_limit = utils::dtor(params->get_parameter("steering_max.pos").as_double());
    lat.weight_lat_error = params->get_parameter("mpc.lateral.weight_lat_error").as_double();
    lat.weight_heading_error = params->get_parameter("mpc.lateral.weight_heading_error").as_double();
    lat.weight_steering_input = params->get_parameter("mpc.lateral.weight_steering_input").as_double();
    lat.weight_steer_rate = params->get_parameter("mpc.lateral.weight_steer_rate").as_double();
    lat.weight_terminal_lat_error =
        params->get_parameter("mpc.lateral.weight_terminal_lat_error").as_double();
    lat.weight_terminal_heading_error =
        params->get_parameter("mpc.lateral.weight_terminal_heading_error").as_double();
    lat.horizon = params->get_parameter("mpc.lateral.horizon").as_int();
    lat.prediction_dt = params->get_parameter("mpc.lateral.prediction_dt").as_double();
    lat.min_predict_speed = params->get_parameter("mpc.lateral.min_predict_speed").as_double();
    lateral_.configure(lat);

    if (lon.v_max <= 0.0 || lat.wheelbase <= 0.0 || lat.steer_limit <= 0.0) {
        throw std::invalid_argument("PidMpcPlugin: control parameters are invalid");
    }

    RCLCPP_INFO(
        logger_,
        "PidMpcPlugin 初期化: 縦PID(kp=%.2f ki=%.2f kd=%.2f) + 横MPC(N=%d dt=%.2f)",
        lon.kp, lon.ki, lon.kd, lat.horizon, lat.prediction_dt);
}

void PidMpcPlugin::reset()
{
    longitudinal_.reset();
    lateral_.reset();
}

void PidMpcPlugin::setMeasuredSteer(double steer)
{
    lateral_.setMeasuredSteer(steer);
}

std::optional<steered_drive_msg::msg::SteeredDrive> PidMpcPlugin::computeCommand(
    const nav_msgs::msg::Path & path_in_base,
    double current_velocity)
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
    const double curvature = forward_max_curvature(path_xy, arc, nearest, 3.0);

    const double v_ref = longitudinal_.referenceSpeed(curvature, dist_to_end);
    const double v_cmd = longitudinal_.update(v_ref, current_velocity);
    const double steer = lateral_.computeSteering(path_xy, current_velocity);

    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = v_cmd;
    command.steering_angle = steer;
    return command;
}

}

PLUGINLIB_EXPORT_CLASS(trajectory_follower::PidMpcPlugin, trajectory_follower::ControllerPlugin)
