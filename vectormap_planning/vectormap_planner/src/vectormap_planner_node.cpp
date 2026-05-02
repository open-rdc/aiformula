#include "vectormap_planner/vectormap_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vectormap_planner
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

VectormapPlannerNode::VectormapPlannerNode(const rclcpp::NodeOptions& options)
: VectormapPlannerNode("", options)
{
}

VectormapPlannerNode::VectormapPlannerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("vectormap_planner_node", name_space, options),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  vector_map_topic_(get_parameter("vector_map_topic").as_string()),
  localization_pose_topic_(get_parameter("localization_pose_topic").as_string()),
  velocity_topic_(get_parameter("velocity_topic").as_string()),
  pointcloud_topic_(get_parameter("pointcloud_topic").as_string()),
  lane_switch_trigger_topic_(get_parameter("lane_switch_trigger_topic").as_string()),
  nav_cmd_topic_(get_parameter("nav_cmd_topic").as_string()),
  default_nav_cmd_(get_parameter("default_nav_cmd").as_string()),
  global_path_topic_(get_parameter("global_path_topic").as_string()),
  local_path_topic_(get_parameter("local_path_topic").as_string()),
  route_lanelet_ids_param_(read_route_lanelet_ids(*this)),
  nav_cmd_fallback_order_param_(read_nav_cmd_fallback_order(*this)),
  global_path_resample_interval_m_(get_parameter("global_path_resample_interval_m").as_double()),
  local_path_horizon_m_(get_parameter("local_path_horizon_m").as_double()),
  local_path_resample_interval_m_(get_parameter("local_path_resample_interval_m").as_double()),
  max_centerline_connection_gap_m_(get_parameter("max_centerline_connection_gap_m").as_double()),
  vehicle_width_m_(get_parameter("vehicle_width_m").as_double()),
  lane_change_length_m_(get_parameter("lane_change_length_m").as_double()),
  avoidance_detection_forward_distance_m_(get_parameter("avoidance_detection_forward_distance_m").as_double()),
  avoidance_hard_margin_m_(get_parameter("avoidance_hard_margin_m").as_double()),
  avoidance_soft_margin_m_(get_parameter("avoidance_soft_margin_m").as_double()),
  envelope_buffer_margin_m_(get_parameter("envelope_buffer_margin_m").as_double()),
  avoidance_lateral_jerk_mps3_(get_parameter("avoidance_lateral_jerk_mps3").as_double()),
  avoidance_min_velocity_mps_(get_parameter("avoidance_min_velocity_mps").as_double()),
  max_avoidance_shift_m_(get_parameter("max_avoidance_shift_m").as_double()),
  frenet_collision_check_margin_m_(get_parameter("frenet_collision_check_margin_m").as_double()),
  frenet_weight_lateral_offset_(get_parameter("frenet_weight_lateral_offset").as_double()),
  frenet_weight_lateral_change_(get_parameter("frenet_weight_lateral_change").as_double()),
  frenet_weight_avoidance_shift_(get_parameter("frenet_weight_avoidance_shift").as_double()),
  obstacle_pointcloud_step_(get_parameter("obstacle_pointcloud_step").as_int()),
  route_lookahead_lanelet_count_(get_parameter("route_lookahead_lanelet_count").as_int()),
  qos_(rclcpp::QoS(10)),
  global_path_ready_(false),
  route_is_loop_(false),
  pending_route_rebuild_(false),
  lane_switch_completed_(false),
  pending_route_rebuild_reason_(""),
  route_version_(0U),
  route_start_lanelet_id_(0U),
  last_nav_cmd_turn_(vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT),
  lane_change_state_(LaneChangeState::Idle),
  active_lane_offset_m_(0.0),
  lane_change_start_s_(0.0),
  lane_change_end_s_(0.0),
  lane_change_start_offset_m_(0.0),
  lane_change_target_offset_m_(0.0)
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }
    if (map_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("frame id parameters must not be empty");
    }
    if (vector_map_topic_.empty() ||
        localization_pose_topic_.empty() ||
        velocity_topic_.empty() ||
        pointcloud_topic_.empty() ||
        lane_switch_trigger_topic_.empty() ||
        nav_cmd_topic_.empty() ||
        global_path_topic_.empty() ||
        local_path_topic_.empty())
    {
        throw std::invalid_argument("topic parameters must not be empty");
    }
    if (global_path_resample_interval_m_ <= 0.0 || local_path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("path resample intervals must be greater than 0");
    }
    if (local_path_horizon_m_ <= local_path_resample_interval_m_) {
        throw std::invalid_argument("local_path_horizon_m must be greater than local_path_resample_interval_m");
    }
    if (max_centerline_connection_gap_m_ < 0.0 || vehicle_width_m_ <= 0.0 || lane_change_length_m_ <= 0.0) {
        throw std::invalid_argument("geometry parameters are invalid");
    }
    if (avoidance_detection_forward_distance_m_ <= 0.0 ||
        avoidance_lateral_jerk_mps3_ <= 0.0 ||
        avoidance_min_velocity_mps_ <= 0.0 ||
        max_avoidance_shift_m_ <= 0.0 ||
        frenet_collision_check_margin_m_ <= 0.0 ||
        frenet_weight_lateral_offset_ < 0.0 ||
        frenet_weight_lateral_change_ < 0.0 ||
        frenet_weight_avoidance_shift_ < 0.0 ||
        obstacle_pointcloud_step_ <= 0)
    {
        throw std::invalid_argument("avoidance parameters are invalid");
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
        std::bind(&VectormapPlannerNode::vector_map_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        localization_pose_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::pose_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        velocity_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::velocity_callback, this, std::placeholders::_1));
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::pointcloud_callback, this, std::placeholders::_1));
    lane_switch_flag_subscription_ = create_subscription<std_msgs::msg::Empty>(
        lane_switch_trigger_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::lane_switch_flag_callback, this, std::placeholders::_1));
    nav_cmd_subscription_ = create_subscription<std_msgs::msg::String>(
        nav_cmd_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::nav_cmd_callback, this, std::placeholders::_1));

    global_path_publisher_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, qos_);
    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, qos_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&VectormapPlannerNode::timer_callback, this));
}

void VectormapPlannerNode::vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("VectorMap message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (global_path_ready_) {
        return;
    }
    build_global_path_once(*msg);
    global_path_publisher_->publish(make_path_message(global_samples_, now()));
}

void VectormapPlannerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void VectormapPlannerNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_ = msg;
}

void VectormapPlannerNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pointcloud_ = msg;
}

void VectormapPlannerNode::lane_switch_flag_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("lane switch flag message must not be null");
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "lane switch request ignored: waiting for vector map route and localization pose");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    const FrenetPoint frenet = project_to_path(ego);
    start_lane_switch(frenet.s, lanelet_at_s(frenet.s));
}

void VectormapPlannerNode::nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
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
            "nav_cmd accepted but route rebuild is pending: waiting for vector map route and localization pose");
        return;
    }
    if (lane_change_state_ == LaneChangeState::Executing || std::abs(active_lane_offset_m_) > 1.0e-6) {
        request_route_rebuild("nav_cmd");
        RCLCPP_WARN(
            get_logger(),
            "nav_cmd accepted and route rebuild is pending because lane switch offset is active");
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    rebuild_route_from_pose(ego, "nav_cmd");
    global_path_publisher_->publish(make_path_message(global_samples_, now()));
}

void VectormapPlannerNode::timer_callback()
{
    std::vector<PathPoint> local_points;
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
        if (pending_route_rebuild_ &&
            lane_change_state_ == LaneChangeState::Idle &&
            std::abs(active_lane_offset_m_) <= 1.0e-6)
        {
            rebuild_route_from_pose(ego, pending_route_rebuild_reason_);
            global_path_publisher_->publish(make_path_message(global_samples_, now()));
            pending_route_rebuild_ = false;
            pending_route_rebuild_reason_.clear();
        }
        FrenetPoint ego_frenet_normalized = project_to_path(ego);
        const uint64_t current_lanelet_id = lanelet_at_s(ego_frenet_normalized.s);
        if (current_lanelet_id != 0U &&
            current_lanelet_id != route_start_lanelet_id_ &&
            lane_change_state_ == LaneChangeState::Idle &&
            std::abs(active_lane_offset_m_) <= 1.0e-6)
        {
            rebuild_route_from_lanelet(current_lanelet_id, "lanelet_crossing");
            global_path_publisher_->publish(make_path_message(global_samples_, now()));
            ego_frenet_normalized = project_to_path(ego);
        }

        double current_s = ego_frenet_normalized.s;
        if (route_is_loop_ &&
            lane_change_state_ == LaneChangeState::Executing &&
            ego_frenet_normalized.s < lane_change_start_s_)
        {
            current_s = ego_frenet_normalized.s + max_path_s();
        }
        const FrenetPoint ego_frenet{current_s, ego_frenet_normalized.d};

        double obstacle_s = std::numeric_limits<double>::quiet_NaN();
        double obstacle_d = std::numeric_limits<double>::quiet_NaN();
        double avoidance_shift = 0.0;
        const bool has_obstacle = latest_pointcloud_ && find_static_obstacle(
            current_s,
            active_lane_offset_m_,
            *latest_pose_,
            *latest_pointcloud_,
            obstacle_s,
            obstacle_d,
            avoidance_shift);

        const double speed = latest_velocity_ ?
            std::hypot(
                latest_velocity_->twist.twist.linear.x,
                latest_velocity_->twist.twist.linear.y) :
            avoidance_min_velocity_mps_;
        local_points = generate_local_path(
            ego_frenet,
            has_obstacle,
            FrenetObstacle{obstacle_s, obstacle_d},
            avoidance_shift,
            std::max(speed, avoidance_min_velocity_mps_));
        if (lane_switch_completed_) {
            rebuild_route_from_pose(ego, pending_route_rebuild_ ? pending_route_rebuild_reason_ : "lane_switch_completed");
            active_lane_offset_m_ = 0.0;
            lane_change_start_offset_m_ = 0.0;
            lane_change_target_offset_m_ = 0.0;
            pending_route_rebuild_ = false;
            pending_route_rebuild_reason_.clear();
            lane_switch_completed_ = false;
            global_path_publisher_->publish(make_path_message(global_samples_, now()));

            const FrenetPoint rebuilt_frenet = project_to_path(ego);
            local_points = generate_local_path(
                rebuilt_frenet,
                has_obstacle,
                FrenetObstacle{obstacle_s, obstacle_d},
                avoidance_shift,
                std::max(speed, avoidance_min_velocity_mps_));
        }
    }

    if (local_points.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "local path is empty; skip publishing");
        return;
    }
    local_path_publisher_->publish(make_path_message(local_points, now()));
}

}  // namespace vectormap_planner
