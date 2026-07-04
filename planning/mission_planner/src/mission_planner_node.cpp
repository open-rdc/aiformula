#include "mission_planner/mission_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mission_planner
{
namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
    const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<std::string> read_nav_cmd_fallback_order(rclcpp::Node& node)
{
    const auto order = node.get_parameter("nav_cmd_fallback_order").as_string_array();
    if (order.empty()) {
        throw std::invalid_argument("nav_cmd_fallback_order must not be empty");
    }
    return order;
}

}

MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions& options)
: MissionPlannerNode("", options)
{
}

MissionPlannerNode::MissionPlannerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("mission_planner_node", name_space, options),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  default_nav_cmd_(get_parameter("default_nav_cmd").as_string()),
  nav_cmd_fallback_order_param_(read_nav_cmd_fallback_order(*this)),
  global_path_resample_interval_m_(get_parameter("global_path_resample_interval_m").as_double()),
  max_centerline_connection_gap_m_(get_parameter("max_centerline_connection_gap_m").as_double()),
  off_route_distance_threshold_m_(get_parameter("off_route_distance_threshold_m").as_double()),
  route_lookahead_lanelet_count_(get_parameter("route_lookahead_lanelet_count").as_int()),
  start_lanelet_yaw_threshold_rad_(get_parameter("start_lanelet_yaw_threshold_rad").as_double()),
  start_pose_position_variance_threshold_(
      get_parameter("start_pose_position_variance_threshold").as_double()),
  qos_(rclcpp::QoS(10)),
  map_ready_(false),
  global_path_ready_(false),
  last_nav_cmd_turn_(vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT)
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }
    if (global_path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("global_path_resample_interval_m must be greater than 0");
    }
    if (max_centerline_connection_gap_m_ < 0.0) {
        throw std::invalid_argument("max_centerline_connection_gap_m must be non-negative");
    }
    if (off_route_distance_threshold_m_ <= 0.0) {
        throw std::invalid_argument("off_route_distance_threshold_m must be greater than 0");
    }
    if (route_lookahead_lanelet_count_ < 3) {
        throw std::invalid_argument("route_lookahead_lanelet_count must be at least 3");
    }
    if (start_lanelet_yaw_threshold_rad_ <= 0.0) {
        throw std::invalid_argument("start_lanelet_yaw_threshold_rad must be greater than 0");
    }
    if (start_pose_position_variance_threshold_ <= 0.0) {
        throw std::invalid_argument("start_pose_position_variance_threshold must be greater than 0");
    }

    last_nav_cmd_turn_ = parse_nav_cmd(default_nav_cmd_);
    nav_cmd_fallback_order_.reserve(nav_cmd_fallback_order_param_.size());
    for (const auto& command : nav_cmd_fallback_order_param_) {
        nav_cmd_fallback_order_.push_back(parse_nav_cmd(command));
    }

    vector_map_subscription_ = create_subscription<vectormap_msgs::msg::VectorMap>(
        "/vector_map",
        qos_,
        std::bind(&MissionPlannerNode::vector_map_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose",
        qos_,
        std::bind(&MissionPlannerNode::pose_callback, this, std::placeholders::_1));
    nav_cmd_subscription_ = create_subscription<std_msgs::msg::String>(
        "/planning/nav_cmd",
        qos_,
        std::bind(&MissionPlannerNode::nav_cmd_callback, this, std::placeholders::_1));
    lane_change_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/flag",
        qos_,
        std::bind(&MissionPlannerNode::lane_change_callback, this, std::placeholders::_1));

    global_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/global_path", qos_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&MissionPlannerNode::timer_callback, this));
}

void MissionPlannerNode::vector_map_callback(
    const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("VectorMap message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (map_ready_) {
        return;
    }
    build_map_lookup(*msg);
}

void MissionPlannerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void MissionPlannerNode::nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("nav_cmd message must not be null");
    }

    uint8_t requested_turn = vectormap_msgs::msg::LaneConnection::TURN_UNKNOWN;
    try {
        requested_turn = parse_nav_cmd(msg->data);
    } catch (const std::invalid_argument& error) {
        RCLCPP_ERROR(get_logger(), "%s", error.what());
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_nav_cmd_turn_ = requested_turn;
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "nav_cmd accepted but route rebuild is pending: waiting for vector map and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    rebuild_route_from_pose(ego, "nav_cmd");
    global_path_publisher_->publish(make_global_path_message(now()));
}

void MissionPlannerNode::lane_change_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("lane change message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "lane change requested but route is not ready: waiting for vector map and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    const auto [current_lanelet_id, distance] = find_nearest_lanelet_within_route(ego);
    if (current_lanelet_id == 0U) {
        RCLCPP_WARN(get_logger(), "lane change rejected: could not determine current lanelet");
        return;
    }

    const bool has_left = left_adjacent_lanelet_by_id_.count(current_lanelet_id) > 0U;
    const bool has_right = right_adjacent_lanelet_by_id_.count(current_lanelet_id) > 0U;
    if (has_left && has_right) {
        RCLCPP_ERROR(
            get_logger(),
            "lane change rejected: both left and right adjacent lanelets exist from %lu",
            current_lanelet_id);
        return;
    }
    if (!has_left && !has_right) {
        RCLCPP_WARN(
            get_logger(),
            "lane change rejected: no adjacent lanelet from %lu",
            current_lanelet_id);
        return;
    }

    const uint64_t adjacent_lanelet_id = has_left
        ? left_adjacent_lanelet_by_id_.at(current_lanelet_id)
        : right_adjacent_lanelet_by_id_.at(current_lanelet_id);

    rebuild_route_from_lanelet(adjacent_lanelet_id, "lane_change");
    global_path_publisher_->publish(make_global_path_message(now()));
}

void MissionPlannerNode::timer_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!map_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "waiting for vector map and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};

    if (!global_path_ready_) {
        const double position_variance_x = latest_pose_->pose.covariance[0];
        const double position_variance_y = latest_pose_->pose.covariance[7];
        if (position_variance_x > start_pose_position_variance_threshold_ ||
            position_variance_y > start_pose_position_variance_threshold_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "waiting for localization pose to converge before building initial route "
                "(var=%.3f,%.3f threshold=%.3f)",
                position_variance_x,
                position_variance_y,
                start_pose_position_variance_threshold_);
            return;
        }

        const double yaw = yaw_from_quaternion(latest_pose_->pose.pose.orientation);
        if (!try_build_initial_route(ego, yaw)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "waiting for a yaw-consistent start lanelet near ego pose");
            return;
        }
        global_path_publisher_->publish(make_global_path_message(now()));
        return;
    }

    const auto [current_lanelet_id, distance] = find_nearest_lanelet_within_route(ego);
    if (current_lanelet_id == 0U) {
        return;
    }

    if (distance > off_route_distance_threshold_m_) {
        rebuild_route_from_pose(ego, "out_of_route");
        global_path_publisher_->publish(make_global_path_message(now()));
        return;
    }

    const auto route_it = std::find(
        current_route_lanelet_ids_.begin(),
        current_route_lanelet_ids_.end(),
        current_lanelet_id);

    const std::size_t remaining = static_cast<std::size_t>(
        std::distance(route_it, current_route_lanelet_ids_.end()));
    if (remaining == 1U) {
        rebuild_route_from_lanelet(current_lanelet_id, "lookahead_extension");
        global_path_publisher_->publish(make_global_path_message(now()));
    }
}

nav_msgs::msg::Path MissionPlannerNode::make_global_path_message(
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = path_frame_id_.empty() ? "map" : path_frame_id_;
    path.poses.reserve(global_samples_.size());
    for (const auto& point : global_samples_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = yaw_to_quaternion(point.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

geometry_msgs::msg::Quaternion MissionPlannerNode::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}
