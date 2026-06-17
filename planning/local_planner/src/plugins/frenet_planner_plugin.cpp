#include "local_planner/plugins/frenet_planner_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace local_planner
{

void FrenetPlannerPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & /*params*/)
{
    logger_ = logger;
}

void FrenetPlannerPlugin::setGlobalPath(const nav_msgs::msg::Path & global_path)
{
    global_path_ = std::make_shared<nav_msgs::msg::Path>(global_path);
}

std::optional<nav_msgs::msg::Path> FrenetPlannerPlugin::computeLocalPath(
    const geometry_msgs::msg::PoseWithCovarianceStamped & /*ego_pose*/,
    const geometry_msgs::msg::TwistWithCovarianceStamped & /*velocity*/,
    const object_detection_msgs::msg::ObjectInfoArray * /*objects*/)
{
    if (!global_path_ || global_path_->poses.empty()) {
        return std::nullopt;
    }
    return *global_path_;
}

}  // namespace local_planner

PLUGINLIB_EXPORT_CLASS(local_planner::FrenetPlannerPlugin, local_planner::LocalPlannerPlugin)
