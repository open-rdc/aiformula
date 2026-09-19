#include "speed_path_planner/speed_path_planner_node.hpp"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>

namespace speed_path_planner
{

namespace
{
speed_profile::Limits read_limits(rclcpp::Node& node)
{
    return speed_profile::Limits{
        node.get_parameter("linear_max.vel").as_double(),
        node.get_parameter("linear_max.acc").as_double(),
        node.get_parameter("a_lat_max").as_double(),
        node.get_parameter("v_min").as_double(),
        node.get_parameter("curvature_window_m").as_double()};
}

std_msgs::msg::ColorRGBA speed_color(double v, double max_speed)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0F;
    if (max_speed > 0.0) {
        color.r = static_cast<float>(std::max(0.0, 1.0 - v / max_speed));
        color.g = static_cast<float>(std::min(1.0, v / max_speed));
        color.b = 0.0F;
    } else {
        color.r = 1.0F;
        color.g = 0.0F;
        color.b = 0.0F;
    }
    return color;
}

speed_path_msgs::msg::SpeedPath make_speed_path(
    const nav_msgs::msg::Path& path, const speed_profile::Profile& profile)
{
    speed_path_msgs::msg::SpeedPath speed_path;
    speed_path.header = path.header;
    speed_path.points.reserve(path.poses.size());
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        speed_path_msgs::msg::SpeedPathPoint point;
        point.pose = path.poses[i].pose;
        point.linear_velocity = profile.velocity[i];
        point.linear_acceleration = profile.acceleration[i];
        point.curvature = profile.curvature[i];
        speed_path.points.push_back(point);
    }
    return speed_path;
}

visualization_msgs::msg::MarkerArray::UniquePtr make_marker_array(
    const speed_path_msgs::msg::SpeedPath& speed_path)
{
    double max_speed = 0.0;
    for (const auto& point : speed_path.points) {
        max_speed = std::max(max_speed, point.linear_velocity);
    }

    visualization_msgs::msg::Marker marker;
    marker.header = speed_path.header;
    marker.ns = "speed_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.points.reserve(speed_path.points.size());
    marker.colors.reserve(speed_path.points.size());
    for (const auto& point : speed_path.points) {
        marker.points.push_back(point.pose.position);
        marker.colors.push_back(speed_color(point.linear_velocity, max_speed));
    }
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.push_back(std::move(marker));
    return marker_array;
}
}

SpeedPathPlannerNode::SpeedPathPlannerNode(const rclcpp::NodeOptions& options)
: SpeedPathPlannerNode("", options)
{
}

SpeedPathPlannerNode::SpeedPathPlannerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("speed_path_planner_node", name_space, options),
  limits_(read_limits(*this)),
  stop_point_timeout_s_(get_parameter("stop_point_timeout_s").as_double())
{
    stop_point_subscription_ = create_subscription<speed_path_msgs::msg::StopPoint>(
        "/planning/stop_point", rclcpp::QoS(1),
        std::bind(&SpeedPathPlannerNode::stop_point_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body", rclcpp::QoS(10),
        std::bind(&SpeedPathPlannerNode::velocity_callback, this, std::placeholders::_1));
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planner/local_path", rclcpp::QoS(1),
        std::bind(&SpeedPathPlannerNode::path_callback, this, std::placeholders::_1));

    speed_path_publisher_ = create_publisher<speed_path_msgs::msg::SpeedPath>("/planner/speed_path", rclcpp::QoS(1));
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/speed_path_visualize", rclcpp::QoS(1));
}

void SpeedPathPlannerNode::stop_point_callback(const speed_path_msgs::msg::StopPoint::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    stop_distance_m_ = msg->distance;
    stop_point_time_ = now();
}

void SpeedPathPlannerNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_velocity_ = msg->twist.twist.linear.x;
}

void SpeedPathPlannerNode::path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    double v_meas;
    double stop_s;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        v_meas = current_velocity_;
        stop_s = (now() - stop_point_time_).seconds() < stop_point_timeout_s_
                     ? stop_distance_m_
                     : std::numeric_limits<double>::infinity();
    }

    std::vector<speed_profile::Point2D> points;
    points.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
        points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    auto speed_path = make_speed_path(*msg, speed_profile::plan(points, v_meas, stop_s, limits_));
    if (marker_publisher_->get_subscription_count() > 0) {
        marker_publisher_->publish(make_marker_array(speed_path));
    }
    speed_path_publisher_->publish(std::make_unique<speed_path_msgs::msg::SpeedPath>(std::move(speed_path)));
}

}
