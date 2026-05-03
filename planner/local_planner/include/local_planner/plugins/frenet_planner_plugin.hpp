#pragma once

#include <memory>
#include <optional>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/obstacle_detect.hpp"
#include "frenet_planner/risk_calculator.hpp"
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
        const sensor_msgs::msg::PointCloud2 * pointcloud) override;

private:
    rclcpp::Logger logger_{rclcpp::get_logger("frenet_planner_plugin")};
    rclcpp::Clock::SharedPtr clock_;

    frenet_planner::RiskCalculator risk_calculator_;
    frenet_planner::FrenetPlanner frenet_planner_{risk_calculator_};
    frenet_planner::ObstacleDetector obstacle_detector_;

    nav_msgs::msg::Path::SharedPtr global_path_;
};

}  // namespace local_planner
