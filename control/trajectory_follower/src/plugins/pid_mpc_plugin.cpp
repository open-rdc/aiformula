#include "trajectory_follower/plugins/pid_mpc_plugin.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "trajectory_follower/speed_path_lookup.hpp"
#include "utilities/utils.hpp"

namespace trajectory_follower
{

void PidMpcPlugin::initialize(
    const rclcpp::Logger &,
    const rclcpp::Clock::SharedPtr &,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    const double dt =
        static_cast<double>(params->get_parameter("interval_ms").as_int()) / 1000.0;

    LongitudinalParams lon;
    lon.v_max = params->get_parameter("linear_max.vel").as_double();
    lon.a_max = params->get_parameter("linear_max.acc").as_double();
    lon.a_min = params->get_parameter("a_min").as_double();
    lon.max_integral_effort = params->get_parameter("mpc.longitudinal.max_integral_effort").as_double();
    lon.jerk_max = params->get_parameter("linear_max.jer").as_double();
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
    lat.weight_terminal_lat_error = params->get_parameter("mpc.lateral.weight_terminal_lat_error").as_double();
    lat.weight_terminal_heading_error = params->get_parameter("mpc.lateral.weight_terminal_heading_error").as_double();
    lat.horizon = params->get_parameter("mpc.lateral.horizon").as_int();
    lat.prediction_dt = params->get_parameter("mpc.lateral.prediction_dt").as_double();
    lat.min_predict_speed = params->get_parameter("mpc.lateral.min_predict_speed").as_double();
    lateral_.configure(lat);

    velocity_preview_time_ = params->get_parameter("velocity_preview_time").as_double();
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

steered_drive_msg::msg::SteeredDrive PidMpcPlugin::computeCommand(
    const speed_path_msgs::msg::SpeedPath & path_in_base,
    double current_velocity)
{
    // 1. 経路を MPC の入力形式に変換する
    const auto & points = path_in_base.points;
    const int n = static_cast<int>(points.size());
    std::vector<std::array<double, 2>> path_xy;
    std::vector<double> curvatures;
    path_xy.reserve(n);
    curvatures.reserve(n);
    for (const auto & point : points) {
        path_xy.push_back({point.pose.position.x, point.pose.position.y});
        curvatures.push_back(point.curvature);
    }

    // 2. 縦方向: 先読み点の速度・加速度を PID で追従する
    const auto reference = preview_point(path_in_base, current_velocity, velocity_preview_time_);

    // 3. 横方向: MPC で舵角を求める
    steered_drive_msg::msg::SteeredDrive command;
    command.velocity = longitudinal_.update(
        reference.linear_velocity, reference.linear_acceleration, current_velocity);
    command.steering_angle = lateral_.computeSteering(path_xy, curvatures, current_velocity);
    return command;
}

}

PLUGINLIB_EXPORT_CLASS(trajectory_follower::PidMpcPlugin, trajectory_follower::ControllerPlugin)
