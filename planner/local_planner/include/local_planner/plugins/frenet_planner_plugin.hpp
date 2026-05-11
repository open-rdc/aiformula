#pragma once

#include <memory>
#include <optional>

#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "local_planner/local_planner_plugin.hpp"

namespace local_planner
{

class FrenetPlannerPlugin : public LocalPlannerPlugin
{
public:
    void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

    void setGlobalPath(const nav_msgs::msg::Path & global_path) override;

    std::optional<nav_msgs::msg::Path> computeLocalPath(
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects) override;

private:
    rclcpp::Logger logger_{rclcpp::get_logger("frenet_planner_plugin")};
    nav_msgs::msg::Path::SharedPtr global_path_;
};

}  // namespace local_planner
