#include "local_planner/local_planner_server.hpp"

#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace local_planner
{

LocalPlannerServer::LocalPlannerServer(const rclcpp::NodeOptions& options)
: rclcpp::Node("local_planner_server_node", options),
  plugin_loader_("local_planner", "local_planner::LocalPlannerPlugin")
{
    plugin_ = plugin_loader_.createSharedInstance(get_parameter("local_planner_plugin").as_string());
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    velocity_ = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    global_path_subscription_ = create_subscription<nav_msgs::msg::Path>("/planner/global_path", 10, std::bind(&LocalPlannerServer::global_path_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vectornav/velocity_body", 10, std::bind(&LocalPlannerServer::velocity_callback, this, std::placeholders::_1));
    objects_subscription_ = create_subscription<object_detection_msgs::msg::ObjectInfoArray>("/perception/objects", rclcpp::SensorDataQoS().keep_last(1), std::bind(&LocalPlannerServer::objects_callback, this, std::placeholders::_1));
    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/local_path", 10);
    stop_point_publisher_ = create_publisher<speed_path_msgs::msg::StopPoint>("/planning/stop_point", rclcpp::QoS(1));

    timer_ = create_wall_timer(std::chrono::milliseconds(get_parameter("interval_ms").as_int()), std::bind(&LocalPlannerServer::timer_callback, this));
}

void LocalPlannerServer::global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    global_path_ = msg;
}

void LocalPlannerServer::velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
    velocity_ = msg;
}

void LocalPlannerServer::objects_callback(const object_detection_msgs::msg::ObjectInfoArray::SharedPtr msg)
{
    if (!tf_buffer_->canTransform("odom", msg->header.frame_id, tf2::TimePointZero)) {
        return;
    }
    const auto transform = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
    objects_ = *msg;
    objects_.header.frame_id = "odom";
    for (auto& object : objects_.objects) {
        geometry_msgs::msg::Point point;
        point.x = object.x;
        point.y = object.y;
        tf2::doTransform(point, point, transform);
        object.x = point.x;
        object.y = point.y;
    }
}

void LocalPlannerServer::timer_callback()
{
    if (!global_path_) {
        return;
    }

    // base_link の経路は観測時刻の姿勢、map の経路は最新の map->odom で odom へ変換する
    const auto path_time = global_path_->header.frame_id == "base_link" ? tf2_ros::fromMsg(global_path_->header.stamp) : tf2::TimePointZero;
    if (!tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero) ||
        !tf_buffer_->canTransform("odom", global_path_->header.frame_id, path_time)) {
        return;
    }
    const auto odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    const auto odom_to_path = tf_buffer_->lookupTransform("odom", global_path_->header.frame_id, path_time);

    nav_msgs::msg::Path global_path;
    global_path.header.stamp = global_path_->header.stamp;
    global_path.header.frame_id = "odom";
    global_path.poses.resize(global_path_->poses.size());
    for (size_t i = 0; i < global_path_->poses.size(); ++i) {
        tf2::doTransform(global_path_->poses[i], global_path.poses[i], odom_to_path);
    }
    plugin_->setGlobalPath(global_path);

    geometry_msgs::msg::Pose ego_pose;
    tf2::doTransform(geometry_msgs::msg::Pose(), ego_pose, odom_to_base);

    const auto local_path = plugin_->computeLocalPath(ego_pose, *velocity_, objects_);
    if (local_path.poses.empty()) {
        return;
    }
    local_path_publisher_->publish(local_path);

    if (std::isfinite(plugin_->stopDistance())) {
        speed_path_msgs::msg::StopPoint stop_point;
        stop_point.distance = plugin_->stopDistance();
        stop_point_publisher_->publish(stop_point);
    }
}

}
