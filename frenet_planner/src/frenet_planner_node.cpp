#include "frenet_planner/frenet_planner_node.hpp"

#include <cmath>

namespace frenet_planner {

FrenetPlannerNode::FrenetPlannerNode(const rclcpp::NodeOptions& options)
    : FrenetPlannerNode("", options) {}

FrenetPlannerNode::FrenetPlannerNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("frenet_planner_node", name_space, options),
  risk_calculator_(),
  frenet_planner_(risk_calculator_),
  obstacle_detector_(),
  wheelbase_(get_parameter("wheelbase").as_double()),
  caster_max_angle_(get_parameter("caster_max_angle").as_double())
{
    double max_curvature = std::tan(caster_max_angle_ * M_PI / 180.0) / wheelbase_;

    frenet_planner_.set_parameters(
        get_parameter("linear_max.vel").as_double(),
        get_parameter("linear_max.acc").as_double(),
        max_curvature,
        get_parameter("dt").as_double(),
        get_parameter("d_t_s").as_double(),
        get_parameter("n_s_sample").as_int(),
        get_parameter("linear_max.vel").as_double(),
        get_parameter("safety_margin").as_double()
    );
    risk_calculator_.set_parameters(
        get_parameter("k_jerk").as_double(),
        get_parameter("k_time").as_double(),
        get_parameter("k_d").as_double(),
        get_parameter("k_s_dot").as_double(),
        get_parameter("k_lat").as_double(),
        get_parameter("k_lon").as_double(),
        get_parameter("linear_max.vel").as_double()
    );

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "e2e_planner/path",
        rclcpp::QoS(10),
        std::bind(&FrenetPlannerNode::path_callback, this, std::placeholders::_1)
    );
    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/pointcloud",
        qos_,
        std::bind(&FrenetPlannerNode::pointcloud_callback, this, std::placeholders::_1)
    );
    sub_velocity_body_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body",
        rclcpp::QoS(10),
        std::bind(&FrenetPlannerNode::velocity_body_callback, this, std::placeholders::_1)
    );

    pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("frenet_planner/path", rclcpp::QoS(10));
    pub_obstacle_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", qos_);

    RCLCPP_INFO(this->get_logger(), "FrenetPlanner node has been initialized.");
}

void FrenetPlannerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pointcloud_ = nullptr;
    pointcloud_ = msg;
}

void FrenetPlannerNode::velocity_body_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg){
    current_twist_ = msg->twist.twist;
}

void FrenetPlannerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
    if (msg->poses.empty()) {
        return;
    }

    global_path_ = msg;
    std::vector<Obstacle> obstacles;
    obstacles = obstacle_detector_.detect_obstacles(pointcloud_);

    local_path_ = obstacles.empty() ? global_path_  : std::make_shared<nav_msgs::msg::Path>(frenet_planner_.plan_local_path(global_path_, obstacles, current_twist_));

    if (!local_path_) {
        local_path_ = global_path_;
    }

    auto marker_array = obstacle_detector_.obstacles_to_marker_array(obstacles, global_path_->header.frame_id, this->now());
    pub_obstacle_markers_->publish(marker_array);
    pub_local_path_->publish(*local_path_);
}

}  // namespace frenet_planner
