#include "mission_planner/mission_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include <vectormap_msgs/msg/lane_connection.hpp>

#include "utilities/utils.hpp"

namespace mission_planner
{
namespace
{

constexpr double EPSILON = 1.0e-6;

std::optional<uint8_t> parse_navigation_command(const std::string& command)
{
    if (command == "straight") {
        return vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT;
    }
    if (command == "left") {
        return vectormap_msgs::msg::LaneConnection::TURN_LEFT;
    }
    if (command == "right") {
        return vectormap_msgs::msg::LaneConnection::TURN_RIGHT;
    }
    return std::nullopt;
}

std::vector<uint8_t> read_navigation_command_fallback_order(rclcpp::Node& node)
{
    const auto order = node.get_parameter("navigation_command_fallback_order").as_string_array();
    if (order.empty()) {
        throw std::invalid_argument("navigation_command_fallback_order must not be empty");
    }
    std::vector<uint8_t> turns;
    turns.reserve(order.size());
    for (const auto& command : order) {
        const auto turn = parse_navigation_command(command);
        if (!turn) {
            throw std::invalid_argument(
                "navigation_command_fallback_order must contain straight, left, or right: " + command);
        }
        turns.push_back(*turn);
    }
    return turns;
}

uint8_t read_default_navigation_command(rclcpp::Node& node)
{
    const auto command = node.get_parameter("default_navigation_command").as_string();
    const auto turn = parse_navigation_command(command);
    if (!turn) {
        throw std::invalid_argument(
            "default_navigation_command must be straight, left, or right: " + command);
    }
    return *turn;
}

double read_curvature_limit_per_m(rclcpp::Node& node)
{
    constexpr double kCurvatureSafetyFactor = 0.8;
    const double wheelbase_m = node.get_parameter("wheelbase").as_double();
    const double steering_max_deg = node.get_parameter("steering_max.pos").as_double();
    return kCurvatureSafetyFactor * std::tan(utils::dtor(steering_max_deg)) / wheelbase_m;
}

double compute_remaining_arc_length_m(
    const std::vector<MissionPlannerNode::PathPoint>& samples,
    const MissionPlannerNode::Point2D& ego)
{
    if (samples.size() < 2U) {
        return 0.0;
    }

    double total_length = 0.0;
    double best_distance_sq = std::numeric_limits<double>::max();
    double best_s = 0.0;
    for (std::size_t i = 1U; i < samples.size(); ++i) {
        const double vx = samples[i].x - samples[i - 1U].x;
        const double vy = samples[i].y - samples[i - 1U].y;
        const double length_sq = vx * vx + vy * vy;
        double t = 0.0;
        if (length_sq > EPSILON) {
            t = std::clamp(
                ((ego.x - samples[i - 1U].x) * vx + (ego.y - samples[i - 1U].y) * vy) / length_sq,
                0.0, 1.0);
        }
        const double dx = ego.x - (samples[i - 1U].x + t * vx);
        const double dy = ego.y - (samples[i - 1U].y + t * vy);
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_s = total_length + t * std::sqrt(length_sq);
        }
        total_length += std::hypot(vx, vy);
    }

    return std::max(0.0, total_length - best_s);
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
  global_path_resample_interval_m_(get_parameter("global_path_resample_interval_m").as_double()),
  max_centerline_connection_gap_m_(get_parameter("max_centerline_connection_gap_m").as_double()),
  off_route_distance_threshold_m_(get_parameter("off_route_distance_threshold_m").as_double()),
  route_lookahead_lanelet_count_(get_parameter("route_lookahead_lanelet_count").as_int()),
  start_lanelet_yaw_threshold_rad_(get_parameter("start_lanelet_yaw_threshold_rad").as_double()),
  start_lanelet_max_distance_m_(get_parameter("start_lanelet_max_distance_m").as_double()),
  start_pose_position_variance_threshold_(get_parameter("start_pose_position_variance_threshold").as_double()),
  route_extension_min_remaining_m_(get_parameter("route_extension_min_remaining_m").as_double()),
  curvature_limit_per_m_(read_curvature_limit_per_m(*this)),
  navigation_command_fallback_order_(read_navigation_command_fallback_order(*this)),
  map_ready_(false),
  global_path_ready_(false),
  current_route_is_loop_(false),
  last_navigation_command_turn_(read_default_navigation_command(*this))
{
    vector_map_subscription_ = create_subscription<vectormap_msgs::msg::VectorMap>(
        "/vector_map",
        rclcpp::QoS(1).transient_local(),
        std::bind(&MissionPlannerNode::vector_map_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MissionPlannerNode::pose_callback, this, std::placeholders::_1));
    navigation_command_subscription_ = create_subscription<std_msgs::msg::String>(
        "/planning/nav_cmd",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&MissionPlannerNode::navigation_command_callback, this, std::placeholders::_1));
    lane_change_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/flag",
        rclcpp::QoS(1),
        std::bind(&MissionPlannerNode::lane_change_callback, this, std::placeholders::_1));

    global_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "/planner/global_path", rclcpp::QoS(1).keep_last(1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&MissionPlannerNode::timer_callback, this));
}

void MissionPlannerNode::vector_map_callback(
    const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
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

void MissionPlannerNode::navigation_command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    const auto requested_turn = parse_navigation_command(msg->data);
    if (!requested_turn) {
        RCLCPP_ERROR(get_logger(), "navigation_commandが不正なため無視する: %s", msg->data.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_navigation_command_turn_ = *requested_turn;
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "navigation_command accepted but route rebuild is pending: waiting for vector map and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    const double yaw = utils::yaw_from_quaternion(latest_pose_->pose.pose.orientation);
    replan_route_from_pose(ego, yaw, "navigation_command");
    global_path_publisher_->publish(make_global_path_message(now()));
}

void MissionPlannerNode::lane_change_callback(const std_msgs::msg::Empty::SharedPtr)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "lane change requested but route is not ready: waiting for vector map and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    const uint64_t current_lanelet_id = find_nearest_lanelet_within_route(ego).first;
    if (current_lanelet_id == 0U) {
        RCLCPP_WARN(get_logger(), "lane change rejected: could not determine current lanelet");
        return;
    }

    const auto left_it = left_adjacent_lanelet_by_id_.find(current_lanelet_id);
    const auto right_it = right_adjacent_lanelet_by_id_.find(current_lanelet_id);
    const bool has_left = left_it != left_adjacent_lanelet_by_id_.end();
    const bool has_right = right_it != right_adjacent_lanelet_by_id_.end();
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

    const uint64_t adjacent_lanelet_id = has_left ? left_it->second : right_it->second;

    replan_route_from_lanelet(adjacent_lanelet_id, "lane_change");
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

        const double yaw = utils::yaw_from_quaternion(latest_pose_->pose.pose.orientation);
        if (!try_start_initial_route(ego, yaw)) {
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
        const double yaw = utils::yaw_from_quaternion(latest_pose_->pose.pose.orientation);
        replan_route_from_pose(ego, yaw, "out_of_route");
        global_path_publisher_->publish(make_global_path_message(now()));
        return;
    }

    if (!current_route_is_loop_) {
        const double remaining_arc_length_m = compute_remaining_arc_length_m(global_samples_, ego);
        if (remaining_arc_length_m < route_extension_min_remaining_m_) {
            replan_route_from_lanelet(current_lanelet_id, "lookahead_extension");
            global_path_publisher_->publish(make_global_path_message(now()));
        }
    }
}

nav_msgs::msg::Path MissionPlannerNode::make_global_path_message(
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    path.poses.reserve(global_samples_.size());
    for (const auto& point : global_samples_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = utils::yaw_to_quaternion(point.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

}
