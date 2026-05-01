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
  lane_change_direction_(get_parameter("lane_change_direction").as_string()),
  vector_map_topic_(get_parameter("vector_map_topic").as_string()),
  localization_pose_topic_(get_parameter("localization_pose_topic").as_string()),
  velocity_topic_(get_parameter("velocity_topic").as_string()),
  pointcloud_topic_(get_parameter("pointcloud_topic").as_string()),
  change_lane_topic_(get_parameter("change_lane_topic").as_string()),
  global_path_topic_(get_parameter("global_path_topic").as_string()),
  local_path_topic_(get_parameter("local_path_topic").as_string()),
  route_lanelet_ids_param_(read_route_lanelet_ids(*this)),
  global_path_resample_interval_m_(get_parameter("global_path_resample_interval_m").as_double()),
  local_path_length_m_(get_parameter("local_path_length_m").as_double()),
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
  obstacle_pointcloud_step_(get_parameter("obstacle_pointcloud_step").as_int()),
  qos_(rclcpp::QoS(10)),
  global_path_ready_(false),
  route_is_loop_(false),
  previous_change_lane_signal_(false),
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
    if (lane_change_direction_ != "left" && lane_change_direction_ != "right") {
        throw std::invalid_argument("lane_change_direction must be left or right");
    }
    if (global_path_resample_interval_m_ <= 0.0 || local_path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("path resample intervals must be greater than 0");
    }
    if (local_path_length_m_ <= local_path_resample_interval_m_) {
        throw std::invalid_argument("local_path_length_m must be greater than local_path_resample_interval_m");
    }
    if (max_centerline_connection_gap_m_ < 0.0 || vehicle_width_m_ <= 0.0 || lane_change_length_m_ <= 0.0) {
        throw std::invalid_argument("geometry parameters are invalid");
    }
    if (avoidance_detection_forward_distance_m_ <= 0.0 ||
        avoidance_lateral_jerk_mps3_ <= 0.0 ||
        avoidance_min_velocity_mps_ <= 0.0 ||
        max_avoidance_shift_m_ <= 0.0 ||
        obstacle_pointcloud_step_ <= 0)
    {
        throw std::invalid_argument("avoidance parameters are invalid");
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
    change_lane_subscription_ = create_subscription<std_msgs::msg::Bool>(
        change_lane_topic_,
        qos_,
        std::bind(&VectormapPlannerNode::change_lane_callback, this, std::placeholders::_1));

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

void VectormapPlannerNode::change_lane_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("change_lane message must not be null");
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    const bool rising_edge = msg->data && !previous_change_lane_signal_;
    previous_change_lane_signal_ = msg->data;
    if (!rising_edge || !global_path_ready_ || !latest_pose_) {
        return;
    }

    const Point2D ego{latest_pose_->pose.pose.position.x, latest_pose_->pose.pose.position.y};
    const FrenetPoint frenet = project_to_path(ego);
    start_lane_change(frenet.s, lanelet_at_s(frenet.s));
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
        const double current_s_normalized = project_to_path(ego).s;
        double current_s = current_s_normalized;
        if (route_is_loop_ &&
            lane_change_state_ == LaneChangeState::Executing &&
            current_s_normalized < lane_change_start_s_)
        {
            current_s += max_path_s();
        }

        double obstacle_s = std::numeric_limits<double>::quiet_NaN();
        double avoidance_shift = 0.0;
        const bool has_obstacle = latest_pointcloud_ && find_static_obstacle(
            current_s,
            active_lane_offset_m_,
            *latest_pose_,
            *latest_pointcloud_,
            obstacle_s,
            avoidance_shift);

        local_points = generate_local_path(current_s);
        if (has_obstacle) {
            const double speed = latest_velocity_ ?
                std::hypot(
                    latest_velocity_->twist.twist.linear.x,
                    latest_velocity_->twist.twist.linear.y) :
                avoidance_min_velocity_mps_;
            const double planning_speed = std::max(speed, avoidance_min_velocity_mps_);
            const double longitudinal_distance =
                4.0 * planning_speed * std::cbrt(0.5 * std::abs(avoidance_shift) / avoidance_lateral_jerk_mps3_);
            const double shift_start_s = std::max(current_s, obstacle_s - longitudinal_distance);
            const double shift_end_s = std::max(shift_start_s + local_path_resample_interval_m_, obstacle_s - 0.5);
            const double return_start_s = obstacle_s + 2.0;
            const double return_end_s = return_start_s + longitudinal_distance;
            local_points = sample_shifted_path(
                current_s,
                route_is_loop_ ? current_s + local_path_length_m_ : std::min(current_s + local_path_length_m_, max_path_s()),
                active_lane_offset_m_,
                lane_change_start_s_,
                lane_change_end_s_,
                lane_change_start_offset_m_,
                lane_change_target_offset_m_,
                obstacle_s,
                avoidance_shift,
                shift_start_s,
                shift_end_s,
                return_start_s,
                return_end_s);
        }
    }

    local_path_publisher_->publish(make_path_message(local_points, now()));
}

}  // namespace vectormap_planner
