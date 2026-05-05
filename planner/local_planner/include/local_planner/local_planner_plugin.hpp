#pragma once

#include <memory>
#include <optional>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vectormap_msgs/msg/vector_map.hpp>

namespace local_planner
{

class LocalPlannerPlugin
{
public:
    using SharedPtr = std::shared_ptr<LocalPlannerPlugin>;
    virtual ~LocalPlannerPlugin() = default;

    virtual void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) = 0;

    virtual void setGlobalPath(const nav_msgs::msg::Path & global_path) = 0;

    virtual void setVectorMap(const vectormap_msgs::msg::VectorMap & /*map*/) {}

    virtual void requestLaneChange() {}

    virtual std::optional<nav_msgs::msg::Path> computeLocalPath(
        const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects) = 0;
};

}  // namespace local_planner
