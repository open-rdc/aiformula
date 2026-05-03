#include "lane_planner/lane_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lane_planner
{
namespace
{

std::vector<int64_t> read_route_lanelet_ids(rclcpp::Node& node)
{
    const auto route = node.get_parameter("route_lanelet_ids").as_integer_array();
    if (route.empty()) {
        throw std::invalid_argument("route_lanelet_ids must not be empty");
    }
    for (const auto id : route) {
        if (id <= 0) {
            throw std::invalid_argument("route_lanelet_ids must contain positive lanelet ids");
        }
    }
    return route;
}

std::vector<std::string> read_nav_cmd_fallback_order(rclcpp::Node& node)
{
    const auto order = node.get_parameter("nav_cmd_fallback_order").as_string_array();
    if (order.empty()) {
        throw std::invalid_argument("nav_cmd_fallback_order must not be empty");
    }
    return order;
}

}  // namespace

LanePlannerNode::LanePlannerNode(const rclcpp::NodeOptions& options)
: LanePlannerNode("", options)
{
}

LanePlannerNode::LanePlannerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("lane_planner_node", name_space, options),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  vector_map_topic_(get_parameter("vector_map_topic").as_string()),
  localization_pose_topic_(get_parameter("localization_pose_topic").as_string()),
  nav_cmd_topic_(get_parameter("nav_cmd_topic").as_string()),
  default_nav_cmd_(get_parameter("default_nav_cmd").as_string()),
  global_path_topic_(get_parameter("global_path_topic").as_string()),
  route_lanelet_ids_param_(read_route_lanelet_ids(*this)),
  nav_cmd_fallback_order_param_(read_nav_cmd_fallback_order(*this)),
  global_path_resample_interval_m_(get_parameter("global_path_resample_interval_m").as_double()),
  max_centerline_connection_gap_m_(get_parameter("max_centerline_connection_gap_m").as_double()),
  route_lookahead_lanelet_count_(get_parameter("route_lookahead_lanelet_count").as_int()),
  qos_(rclcpp::QoS(10)),
  global_path_ready_(false),
  route_is_loop_(false),
  pending_route_rebuild_(false),
  pending_route_rebuild_reason_(""),
  last_nav_cmd_turn_(vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT)
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }
    if (map_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("frame id parameters must not be empty");
    }
    if (vector_map_topic_.empty() ||
        localization_pose_topic_.empty() ||
        nav_cmd_topic_.empty() ||
        global_path_topic_.empty())
    {
        throw std::invalid_argument("topic parameters must not be empty");
    }
    if (global_path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("global_path_resample_interval_m must be greater than 0");
    }
    if (max_centerline_connection_gap_m_ < 0.0) {
        throw std::invalid_argument("max_centerline_connection_gap_m must be non-negative");
    }
    if (route_lookahead_lanelet_count_ < 3) {
        throw std::invalid_argument("route_lookahead_lanelet_count must be at least 3");
    }

    last_nav_cmd_turn_ = parse_nav_cmd(default_nav_cmd_);
    nav_cmd_fallback_order_.reserve(nav_cmd_fallback_order_param_.size());
    for (const auto& command : nav_cmd_fallback_order_param_) {
        nav_cmd_fallback_order_.push_back(parse_nav_cmd(command));
    }

    vector_map_subscription_ = create_subscription<vectormap_msgs::msg::VectorMap>(
        vector_map_topic_,
        qos_,
        std::bind(&LanePlannerNode::vector_map_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        localization_pose_topic_,
        qos_,
        std::bind(&LanePlannerNode::pose_callback, this, std::placeholders::_1));
    nav_cmd_subscription_ = create_subscription<std_msgs::msg::String>(
        nav_cmd_topic_,
        qos_,
        std::bind(&LanePlannerNode::nav_cmd_callback, this, std::placeholders::_1));

    global_path_publisher_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, qos_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&LanePlannerNode::timer_callback, this));
}

void LanePlannerNode::vector_map_callback(
    const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("VectorMap message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (global_path_ready_) {
        return;
    }
    build_global_path_once(*msg);
    global_path_publisher_->publish(make_global_path_message(now()));
}

void LanePlannerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void LanePlannerNode::nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
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

void LanePlannerNode::timer_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "waiting for vector map route and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};

    if (pending_route_rebuild_) {
        rebuild_route_from_pose(ego, pending_route_rebuild_reason_);
        global_path_publisher_->publish(make_global_path_message(now()));
        pending_route_rebuild_ = false;
        pending_route_rebuild_reason_.clear();
        return;
    }

    const uint64_t current_lanelet_id = find_nearest_lanelet_from_pose(ego);
    if (current_lanelet_id == 0U) {
        return;
    }

    const auto route_it = std::find(
        current_route_lanelet_ids_.begin(),
        current_route_lanelet_ids_.end(),
        current_lanelet_id);

    if (route_it == current_route_lanelet_ids_.end()) {
        // Vehicle is not on any planned lanelet
        rebuild_route_from_lanelet(current_lanelet_id, "out_of_route");
        global_path_publisher_->publish(make_global_path_message(now()));
    } else {
        // Rebuild only when the vehicle reaches the last lanelet in the route
        const std::size_t remaining = static_cast<std::size_t>(
            std::distance(route_it, current_route_lanelet_ids_.end()));
        if (remaining == 1U) {
            rebuild_route_from_lanelet(current_lanelet_id, "lookahead_extension");
            global_path_publisher_->publish(make_global_path_message(now()));
        }
    }
}

nav_msgs::msg::Path LanePlannerNode::make_global_path_message(
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = path_frame_id_.empty() ? map_frame_id_ : path_frame_id_;
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

geometry_msgs::msg::Quaternion LanePlannerNode::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace lane_planner
