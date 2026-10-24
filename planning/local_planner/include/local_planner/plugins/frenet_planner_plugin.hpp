#pragma once

#include <limits>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "local_planner/frenet/frenet_planner.hpp"
#include "local_planner/frenet/reference_path.hpp"
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

    nav_msgs::msg::Path computeLocalPath(
        const geometry_msgs::msg::Pose & ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
        const object_detection_msgs::msg::ObjectInfoArray & objects) override;

    double stopDistance() const override;

private:
    frenet::Parameters params_;
    frenet::ReferencePath reference_;
    std_msgs::msg::Header global_path_header_;
    std::vector<frenet::Pose2D> previous_points_;
    double stop_distance_m_{std::numeric_limits<double>::infinity()};
};

}
