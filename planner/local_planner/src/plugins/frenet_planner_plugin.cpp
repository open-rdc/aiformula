#include "local_planner/plugins/frenet_planner_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace local_planner
{

void FrenetPlannerPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    const double max_speed = params->get_parameter("linear_max.vel").get_value<double>();
    const double max_accel = params->get_parameter("linear_max.acc").get_value<double>();
    const double max_curvature = params->get_parameter("steering_max.pos").get_value<double>();
    const double dt = params->get_parameter("dt").get_value<double>();
    const double d_t_s = params->get_parameter("d_t_s").get_value<double>();
    const int n_s_sample = params->get_parameter("n_s_sample").get_value<int>();
    const double safety_margin = params->get_parameter("safety_margin").get_value<double>();

    frenet_planner_.set_parameters(
        max_speed, max_accel, max_curvature, dt, d_t_s, n_s_sample, max_speed, safety_margin);

    const double k_jerk = params->get_parameter("k_jerk").get_value<double>();
    const double k_time = params->get_parameter("k_time").get_value<double>();
    const double k_d = params->get_parameter("k_d").get_value<double>();
    const double k_d_s = params->get_parameter("k_d_s").get_value<double>();
    const double k_s_dot = params->get_parameter("k_s_dot").get_value<double>();
    const double k_lat = params->get_parameter("k_lat").get_value<double>();
    const double k_lon = params->get_parameter("k_lon").get_value<double>();

    risk_calculator_.set_parameters(k_jerk, k_time, k_d, k_d_s, k_s_dot, k_lat, k_lon, max_speed);
}

void FrenetPlannerPlugin::setGlobalPath(const nav_msgs::msg::Path & global_path)
{
    global_path_ = std::make_shared<nav_msgs::msg::Path>(global_path);
}

std::optional<nav_msgs::msg::Path> FrenetPlannerPlugin::computeLocalPath(
    const geometry_msgs::msg::PoseWithCovarianceStamped & /*ego_pose*/,
    const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
    const sensor_msgs::msg::PointCloud2 * pointcloud)
{
    if (!global_path_ || global_path_->poses.empty()) {
        return std::nullopt;
    }
    if (!pointcloud) {
        return *global_path_;
    }

    const auto pc_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(*pointcloud);
    const auto obstacles = obstacle_detector_.detect_obstacles(pc_ptr);
    if (obstacles.empty()) {
        return *global_path_;
    }

    geometry_msgs::msg::Twist twist = velocity.twist.twist;
    return frenet_planner_.plan_local_path(global_path_, obstacles, twist);
}

}  // namespace local_planner

PLUGINLIB_EXPORT_CLASS(local_planner::FrenetPlannerPlugin, local_planner::LocalPlannerPlugin)
