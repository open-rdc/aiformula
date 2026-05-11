#include "local_planner/plugins/vectormap_frenet_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <vectormap_msgs/msg/lanelet.hpp>
#include <vectormap_msgs/msg/line_string.hpp>

namespace local_planner
{
namespace
{
constexpr double EPSILON = 1.0e-6;
}  // namespace

void VectormapFrenetPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    local_path_horizon_m_ =
        params->get_parameter("local_path_horizon_m").get_value<double>();
    local_path_resample_interval_m_ =
        params->get_parameter("local_path_resample_interval_m").get_value<double>();
    max_centerline_connection_gap_m_ =
        params->get_parameter("max_centerline_connection_gap_m").get_value<double>();
    vehicle_width_m_ =
        params->get_parameter("vehicle_width_m").get_value<double>();
    lane_change_length_m_ =
        params->get_parameter("lane_change_length_m").get_value<double>();
    avoidance_detection_forward_distance_m_ =
        params->get_parameter("avoidance_detection_forward_distance_m").get_value<double>();
    avoidance_hard_margin_m_ =
        params->get_parameter("avoidance_hard_margin_m").get_value<double>();
    avoidance_soft_margin_m_ =
        params->get_parameter("avoidance_soft_margin_m").get_value<double>();
    envelope_buffer_margin_m_ =
        params->get_parameter("envelope_buffer_margin_m").get_value<double>();
    avoidance_lateral_jerk_mps3_ =
        params->get_parameter("avoidance_lateral_jerk_mps3").get_value<double>();
    avoidance_min_velocity_mps_ =
        params->get_parameter("avoidance_min_velocity_mps").get_value<double>();
    max_avoidance_shift_m_ =
        params->get_parameter("max_avoidance_shift_m").get_value<double>();
    frenet_collision_check_margin_m_ =
        params->get_parameter("frenet_collision_check_margin_m").get_value<double>();
    frenet_weight_lateral_offset_ =
        params->get_parameter("frenet_weight_lateral_offset").get_value<double>();
    frenet_weight_lateral_change_ =
        params->get_parameter("frenet_weight_lateral_change").get_value<double>();
    frenet_weight_avoidance_shift_ =
        params->get_parameter("frenet_weight_avoidance_shift").get_value<double>();
    map_frame_id_ =
        params->get_parameter("map_frame_id").get_value<std::string>();
    base_frame_id_ =
        params->get_parameter("base_frame_id").get_value<std::string>();

    if (local_path_horizon_m_ <= local_path_resample_interval_m_) {
        throw std::invalid_argument(
            "local_path_horizon_m must be greater than local_path_resample_interval_m");
    }
    if (vehicle_width_m_ <= 0.0 || lane_change_length_m_ <= 0.0 ||
        avoidance_detection_forward_distance_m_ <= 0.0 ||
        avoidance_lateral_jerk_mps3_ <= 0.0 ||
        avoidance_min_velocity_mps_ <= 0.0 ||
        max_avoidance_shift_m_ <= 0.0 ||
        frenet_collision_check_margin_m_ <= 0.0)
    {
        throw std::invalid_argument("VectormapFrenetPlugin parameters are invalid");
    }
}

void VectormapFrenetPlugin::setGlobalPath(const nav_msgs::msg::Path & global_path)
{
    if (global_path.poses.empty()) {
        return;
    }

    global_samples_.clear();
    global_samples_.reserve(global_path.poses.size());
    path_frame_id_ = global_path.header.frame_id;

    double s = 0.0;
    for (std::size_t i = 0U; i < global_path.poses.size(); ++i) {
        const auto& pose = global_path.poses[i];
        if (i > 0U) {
            const auto& prev = global_path.poses[i - 1U];
            const double dx = pose.pose.position.x - prev.pose.position.x;
            const double dy = pose.pose.position.y - prev.pose.position.y;
            s += std::hypot(dx, dy);
        }
        const double yaw = yaw_from_quaternion(pose.pose.orientation);
        global_samples_.push_back(PathPoint{s, pose.pose.position.x, pose.pose.position.y, yaw, 0U});
    }

    const auto& first = global_samples_.front();
    const auto& last = global_samples_.back();
    route_is_loop_ =
        std::hypot(last.x - first.x, last.y - first.y) < max_centerline_connection_gap_m_;

    // Reset lane change state whenever global path is updated
    lane_change_state_ = LaneChangeState::Idle;
    active_lane_offset_m_ = 0.0;
    lane_change_start_offset_m_ = 0.0;
    lane_change_target_offset_m_ = 0.0;
    lane_change_requested_ = false;
    lane_switch_completed_ = false;

    global_path_ready_ = true;
}

void VectormapFrenetPlugin::setVectorMap(const vectormap_msgs::msg::VectorMap & map)
{
    using Lanelet = vectormap_msgs::msg::Lanelet;
    using LineString = vectormap_msgs::msg::LineString;

    std::unordered_map<uint64_t, Lanelet> lanelet_by_id;
    lanelet_by_id.reserve(map.lanelets.size());
    for (const auto& lanelet : map.lanelets) {
        lanelet_by_id.emplace(lanelet.id, lanelet);
    }

    std::unordered_map<uint64_t, LineString> line_string_by_id;
    line_string_by_id.reserve(map.line_strings.size());
    for (const auto& line_string : map.line_strings) {
        line_string_by_id.emplace(line_string.id, line_string);
    }

    lanelet_centerline_points_by_id_.clear();
    lanelet_centerline_points_by_id_.reserve(lanelet_by_id.size());
    for (const auto& [lanelet_id, lanelet] : lanelet_by_id) {
        const auto line_it = line_string_by_id.find(lanelet.centerline_id);
        if (line_it == line_string_by_id.end() || line_it->second.points.size() < 2U) {
            continue;
        }
        std::vector<Point2D> centerline_points;
        centerline_points.reserve(line_it->second.points.size());
        for (const auto& point : line_it->second.points) {
            centerline_points.push_back(Point2D{point.x, point.y});
        }
        lanelet_centerline_points_by_id_.emplace(lanelet_id, std::move(centerline_points));
    }

    left_adjacent_lanelet_by_id_.clear();
    right_adjacent_lanelet_by_id_.clear();
    for (const auto& [id, lanelet] : lanelet_by_id) {
        for (const auto& [other_id, other] : lanelet_by_id) {
            if (id == other_id) {
                continue;
            }
            if (lanelet.left_line_id == other.right_line_id) {
                left_adjacent_lanelet_by_id_[id] = other_id;
            }
            if (lanelet.right_line_id == other.left_line_id) {
                right_adjacent_lanelet_by_id_[id] = other_id;
            }
        }
    }
}

void VectormapFrenetPlugin::requestLaneChange()
{
    lane_change_requested_ = true;
}

std::optional<nav_msgs::msg::Path> VectormapFrenetPlugin::computeLocalPath(
    const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
    const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
    const object_detection_msgs::msg::ObjectInfoArray * objects)
{
    if (!global_path_ready_) {
        return std::nullopt;
    }

    const Point2D ego{ego_pose.pose.pose.position.x, ego_pose.pose.pose.position.y};

    if (lane_change_requested_ && lane_change_state_ == LaneChangeState::Idle) {
        const FrenetPoint frenet = project_to_path(ego);
        start_lane_switch(frenet.s, ego);
        lane_change_requested_ = false;
    }

    FrenetPoint ego_frenet = project_to_path(ego);
    if (route_is_loop_ &&
        lane_change_state_ == LaneChangeState::Executing &&
        ego_frenet.s < lane_change_start_s_)
    {
        ego_frenet.s += max_path_s();
    }

    double obstacle_s = std::numeric_limits<double>::quiet_NaN();
    double obstacle_d = std::numeric_limits<double>::quiet_NaN();
    double avoidance_shift = 0.0;
    const bool has_obstacle = objects && !objects->objects.empty() && find_static_obstacle(
        ego_frenet.s,
        active_lane_offset_m_,
        *objects,
        obstacle_s,
        obstacle_d,
        avoidance_shift);

    const double speed = std::hypot(
        velocity.twist.twist.linear.x, velocity.twist.twist.linear.y);
    const double effective_speed = std::max(speed, avoidance_min_velocity_mps_);

    auto local_points = generate_local_path(
        ego_frenet,
        has_obstacle,
        FrenetObstacle{obstacle_s, obstacle_d},
        avoidance_shift,
        effective_speed);

    if (lane_switch_completed_) {
        active_lane_offset_m_ = 0.0;
        lane_change_start_offset_m_ = 0.0;
        lane_change_target_offset_m_ = 0.0;
        lane_switch_completed_ = false;

        const FrenetPoint rebuilt_frenet = project_to_path(ego);
        local_points = generate_local_path(
            rebuilt_frenet,
            has_obstacle,
            FrenetObstacle{obstacle_s, obstacle_d},
            avoidance_shift,
            effective_speed);
    }

    if (local_points.empty()) {
        return std::nullopt;
    }
    return make_path_message(local_points, clock_->now());
}

void VectormapFrenetPlugin::start_lane_switch(
    const double current_s,
    const Point2D& ego)
{
    if (std::abs(active_lane_offset_m_) > EPSILON) {
        start_lane_offset_transition(current_s, 0.0, "return_to_route_lane");
        return;
    }

    const uint64_t current_lanelet_id = find_nearest_lanelet(ego);
    if (current_lanelet_id == 0U) {
        RCLCPP_WARN(logger_, "lane switch rejected: could not determine current lanelet");
        return;
    }

    const bool has_left =
        left_adjacent_lanelet_by_id_.find(current_lanelet_id) != left_adjacent_lanelet_by_id_.end();
    const bool has_right =
        right_adjacent_lanelet_by_id_.find(current_lanelet_id) != right_adjacent_lanelet_by_id_.end();

    if (has_left && has_right) {
        RCLCPP_ERROR(
            logger_,
            "lane switch rejected: both left and right adjacent lanelets exist from %lu;"
            " expected two-lane route",
            current_lanelet_id);
        return;
    }
    if (!has_left && !has_right) {
        RCLCPP_WARN(
            logger_,
            "lane switch rejected: no adjacent lanelet from %lu",
            current_lanelet_id);
        return;
    }

    const bool to_left = has_left;
    const double target_offset = adjacent_lane_offset(current_lanelet_id, to_left);
    if (!std::isfinite(target_offset) || std::abs(target_offset) <= EPSILON) {
        RCLCPP_WARN(logger_, "lane switch rejected: adjacent lane offset is invalid");
        return;
    }

    start_lane_offset_transition(current_s, target_offset, "switch_to_adjacent_lane");
}

void VectormapFrenetPlugin::start_lane_offset_transition(
    const double current_s,
    const double target_offset,
    const std::string& reason)
{
    if (lane_change_state_ == LaneChangeState::Executing) {
        RCLCPP_WARN(logger_, "lane switch request ignored because another lane change is executing");
        return;
    }
    if (!std::isfinite(target_offset)) {
        throw std::runtime_error("lane switch target offset must be finite");
    }
    if (std::abs(target_offset - active_lane_offset_m_) <= EPSILON) {
        RCLCPP_WARN(logger_, "lane switch request ignored because target offset is already active");
        return;
    }

    lane_change_state_ = LaneChangeState::Executing;
    lane_change_start_s_ = current_s;
    lane_change_end_s_ = route_is_loop_ ?
        current_s + lane_change_length_m_ :
        std::min(current_s + lane_change_length_m_, max_path_s());
    lane_change_start_offset_m_ = active_lane_offset_m_;
    lane_change_target_offset_m_ = target_offset;
    RCLCPP_INFO(
        logger_,
        "lane switch started: reason=%s start_offset=%.3f target_offset=%.3f",
        reason.c_str(),
        lane_change_start_offset_m_,
        lane_change_target_offset_m_);
}

std::vector<VectormapFrenetPlugin::PathPoint> VectormapFrenetPlugin::generate_local_path(
    const FrenetPoint& ego_frenet,
    const bool has_obstacle,
    const FrenetObstacle& obstacle,
    const double avoidance_shift,
    const double speed_mps)
{
    const double current_s = ego_frenet.s;
    double target_offset = active_lane_offset_m_;
    double lane_start_offset = active_lane_offset_m_;
    double lane_end_offset = active_lane_offset_m_;
    double start_s = lane_change_start_s_;
    double end_s = lane_change_end_s_;

    if (lane_change_state_ == LaneChangeState::Executing && current_s >= lane_change_end_s_) {
        active_lane_offset_m_ = lane_change_target_offset_m_;
        lane_change_state_ = LaneChangeState::Idle;
        lane_switch_completed_ = true;
        target_offset = active_lane_offset_m_;
        lane_start_offset = active_lane_offset_m_;
        lane_end_offset = active_lane_offset_m_;
        start_s = current_s;
        end_s = current_s;
        RCLCPP_INFO(
            logger_,
            "lane change completed: active_offset=%.3f",
            active_lane_offset_m_);
    } else if (lane_change_state_ == LaneChangeState::Executing) {
        target_offset = lane_change_target_offset_m_;
        lane_start_offset = lane_change_start_offset_m_;
        lane_end_offset = lane_change_target_offset_m_;
    }

    const double path_end_s = route_is_loop_ ?
        current_s + local_path_horizon_m_ :
        std::min(current_s + local_path_horizon_m_, max_path_s());
    if (path_end_s <= current_s + EPSILON) {
        return {};
    }

    double avoidance_start_s = 0.0;
    double avoidance_end_s = 0.0;
    double avoidance_return_start_s = 0.0;
    double avoidance_return_end_s = 0.0;
    if (has_obstacle) {
        const double longitudinal_distance =
            4.0 * speed_mps * std::cbrt(0.5 * std::abs(avoidance_shift) / avoidance_lateral_jerk_mps3_);
        avoidance_start_s = std::max(current_s, obstacle.s - longitudinal_distance);
        avoidance_end_s =
            std::max(avoidance_start_s + local_path_resample_interval_m_, obstacle.s - 0.5);
        avoidance_return_start_s = obstacle.s + 2.0;
        avoidance_return_end_s = avoidance_return_start_s + longitudinal_distance;
    }

    std::vector<double> candidate_shifts;
    candidate_shifts.reserve(3U);
    candidate_shifts.push_back(has_obstacle ? avoidance_shift : 0.0);
    if (has_obstacle) {
        candidate_shifts.push_back(-avoidance_shift);
        candidate_shifts.push_back(0.0);
    }

    double best_cost = std::numeric_limits<double>::max();
    std::vector<PathPoint> best_path;
    for (const double candidate_shift : candidate_shifts) {
        auto candidate = sample_frenet_path(
            current_s, path_end_s, ego_frenet.d,
            start_s, end_s, lane_start_offset, lane_end_offset, target_offset,
            obstacle, candidate_shift,
            avoidance_start_s, avoidance_end_s,
            avoidance_return_start_s, avoidance_return_end_s);
        if (candidate.empty()) {
            continue;
        }
        if (has_obstacle && !is_collision_free(candidate, obstacle)) {
            continue;
        }
        const double cost = evaluate_frenet_candidate(candidate, target_offset, candidate_shift);
        if (cost < best_cost) {
            best_cost = cost;
            best_path = std::move(candidate);
        }
    }
    return best_path;
}

std::vector<VectormapFrenetPlugin::PathPoint> VectormapFrenetPlugin::sample_frenet_path(
    const double start_s,
    const double end_s,
    const double start_d,
    const double lane_change_start_s,
    const double lane_change_end_s,
    const double lane_change_start_offset,
    const double lane_change_end_offset,
    const double fallback_target_offset,
    const FrenetObstacle& obstacle,
    const double avoidance_shift,
    const double avoidance_start_s,
    const double avoidance_end_s,
    const double avoidance_return_start_s,
    const double avoidance_return_end_s) const
{
    std::vector<PathPoint> points;
    if (end_s <= start_s) {
        return points;
    }
    points.reserve(
        static_cast<std::size_t>(
            std::ceil((end_s - start_s) / local_path_resample_interval_m_)) + 2U);

    const double convergence_length =
        std::min(5.0, std::max(local_path_resample_interval_m_, end_s - start_s));
    for (double s = start_s; s < end_s; s += local_path_resample_interval_m_) {
        const auto base_point = path_point_at_s(s);
        double target_offset = fallback_target_offset;
        if (lane_change_end_s > lane_change_start_s && s >= lane_change_start_s) {
            const double t =
                std::clamp((s - lane_change_start_s) / (lane_change_end_s - lane_change_start_s),
                           0.0, 1.0);
            target_offset = lane_change_start_offset +
                smooth_step(t) * (lane_change_end_offset - lane_change_start_offset);
        }

        const double converge_t =
            std::clamp((s - start_s) / convergence_length, 0.0, 1.0);
        double offset = start_d + smooth_step(converge_t) * (target_offset - start_d);
        if (std::isfinite(obstacle.s)) {
            if (s >= avoidance_start_s && s < avoidance_end_s) {
                const double t = std::clamp(
                    (s - avoidance_start_s) / (avoidance_end_s - avoidance_start_s), 0.0, 1.0);
                offset += smooth_step(t) * avoidance_shift;
            } else if (s >= avoidance_end_s && s < avoidance_return_start_s) {
                offset += avoidance_shift;
            } else if (s >= avoidance_return_start_s && s < avoidance_return_end_s) {
                const double t = std::clamp(
                    (s - avoidance_return_start_s) /
                    (avoidance_return_end_s - avoidance_return_start_s),
                    0.0, 1.0);
                offset += (1.0 - smooth_step(t)) * avoidance_shift;
            }
        }

        points.push_back(PathPoint{
            s,
            base_point.x - std::sin(base_point.yaw) * offset,
            base_point.y + std::cos(base_point.yaw) * offset,
            base_point.yaw,
            0U});
    }

    if (!points.empty() && end_s - points.back().s > EPSILON) {
        const auto base_point = path_point_at_s(end_s);
        const double offset = points.back().s < end_s ?
            project_to_path(Point2D{points.back().x, points.back().y}).d :
            start_d;
        points.push_back(PathPoint{
            end_s,
            base_point.x - std::sin(base_point.yaw) * offset,
            base_point.y + std::cos(base_point.yaw) * offset,
            base_point.yaw,
            0U});
    }

    for (std::size_t i = 1U; i < points.size(); ++i) {
        const double dx = points[i].x - points[i - 1U].x;
        const double dy = points[i].y - points[i - 1U].y;
        if (std::hypot(dx, dy) > EPSILON) {
            points[i - 1U].yaw = std::atan2(dy, dx);
        }
    }
    if (points.size() >= 2U) {
        points.back().yaw = points[points.size() - 2U].yaw;
    }
    return points;
}

bool VectormapFrenetPlugin::is_collision_free(
    const std::vector<PathPoint>& candidate,
    const FrenetObstacle& obstacle) const
{
    if (!std::isfinite(obstacle.s) || !std::isfinite(obstacle.d)) {
        return true;
    }
    const double longitudinal_margin =
        std::max(0.5, local_path_resample_interval_m_ * 2.0);
    const double lateral_margin =
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ +
        envelope_buffer_margin_m_ + frenet_collision_check_margin_m_;
    for (const auto& point : candidate) {
        if (std::abs(point.s - obstacle.s) > longitudinal_margin) {
            continue;
        }
        const FrenetPoint frenet = project_to_path(Point2D{point.x, point.y});
        if (std::abs(frenet.d - obstacle.d) <= lateral_margin) {
            return false;
        }
    }
    return true;
}

double VectormapFrenetPlugin::evaluate_frenet_candidate(
    const std::vector<PathPoint>& candidate,
    const double target_offset,
    const double avoidance_shift) const
{
    if (candidate.empty()) {
        return std::numeric_limits<double>::max();
    }

    double lateral_change_sum = 0.0;
    double previous_d =
        project_to_path(Point2D{candidate.front().x, candidate.front().y}).d;
    for (std::size_t i = 1U; i < candidate.size(); ++i) {
        const double d = project_to_path(Point2D{candidate[i].x, candidate[i].y}).d;
        lateral_change_sum += std::abs(d - previous_d);
        previous_d = d;
    }
    const double final_d = previous_d;
    return frenet_weight_lateral_offset_ * std::abs(final_d - target_offset) +
        frenet_weight_lateral_change_ * lateral_change_sum +
        frenet_weight_avoidance_shift_ * std::abs(avoidance_shift);
}

bool VectormapFrenetPlugin::find_static_obstacle(
    const double current_s,
    const double base_offset,
    const object_detection_msgs::msg::ObjectInfoArray& objects,
    double& obstacle_s,
    double& obstacle_d,
    double& avoidance_shift) const
{
    const double lateral_limit =
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ +
        avoidance_soft_margin_m_ + envelope_buffer_margin_m_;
    const double current_s_normalized = normalize_path_s(current_s);

    bool found = false;
    double best_s = std::numeric_limits<double>::max();
    double d_sum = 0.0;
    std::size_t d_count = 0U;

    for (const auto& obj : objects.objects) {
        const FrenetPoint frenet = project_to_path(Point2D{obj.x, obj.y});
        double delta_s = frenet.s - current_s_normalized;
        if (route_is_loop_ && delta_s < 0.0) {
            delta_s += max_path_s();
        }
        if (delta_s < 0.0 || delta_s > avoidance_detection_forward_distance_m_) {
            continue;
        }
        if (std::abs(frenet.d - base_offset) > lateral_limit) {
            continue;
        }
        found = true;
        best_s = std::min(best_s, current_s + delta_s);
        d_sum += frenet.d;
        ++d_count;
    }

    if (!found || d_count == 0U) {
        return false;
    }

    obstacle_s = best_s;
    obstacle_d = d_sum / static_cast<double>(d_count);
    const double shift_sign = obstacle_d >= base_offset ? -1.0 : 1.0;
    const double required_shift =
        std::min(max_avoidance_shift_m_,
                 vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ + avoidance_soft_margin_m_);
    avoidance_shift = shift_sign * required_shift;
    return true;
}

VectormapFrenetPlugin::FrenetPoint VectormapFrenetPlugin::project_to_path(
    const Point2D& point) const
{
    if (global_samples_.empty()) {
        throw std::runtime_error("global path is not ready");
    }

    double best_distance_sq = std::numeric_limits<double>::max();
    FrenetPoint best{global_samples_.front().s, 0.0};
    for (std::size_t i = 1U; i < global_samples_.size(); ++i) {
        const auto& a = global_samples_[i - 1U];
        const auto& b = global_samples_[i];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double length_sq = vx * vx + vy * vy;
        if (length_sq <= EPSILON) {
            continue;
        }
        const double wx = point.x - a.x;
        const double wy = point.y - a.y;
        const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
        const double px = a.x + t * vx;
        const double py = a.y + t * vy;
        const double dx = point.x - px;
        const double dy = point.y - py;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            const double yaw = std::atan2(vy, vx);
            best.s = a.s + t * (b.s - a.s);
            best.d = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        }
    }
    return best;
}

VectormapFrenetPlugin::PathPoint VectormapFrenetPlugin::path_point_at_s(
    const double s) const
{
    if (global_samples_.size() < 2U) {
        throw std::runtime_error("global path is not ready");
    }

    const double clamped_s = normalize_path_s(s);
    const auto upper = std::upper_bound(
        global_samples_.begin(),
        global_samples_.end(),
        clamped_s,
        [](const double value, const PathPoint& point) {
            return value < point.s;
        });
    std::size_t index = 0U;
    if (upper == global_samples_.end()) {
        index = global_samples_.size() - 2U;
    } else if (upper != global_samples_.begin()) {
        index = static_cast<std::size_t>(
            std::distance(global_samples_.begin(), upper) - 1);
    }

    const auto& start = global_samples_[index];
    const auto& end = global_samples_[index + 1U];
    const double segment_length = end.s - start.s;
    const double ratio = segment_length > EPSILON ? (clamped_s - start.s) / segment_length : 0.0;
    return PathPoint{
        clamped_s,
        start.x + ratio * (end.x - start.x),
        start.y + ratio * (end.y - start.y),
        std::atan2(end.y - start.y, end.x - start.x),
        0U};
}

double VectormapFrenetPlugin::max_path_s() const
{
    if (global_samples_.empty()) {
        throw std::runtime_error("global path is not ready");
    }
    return global_samples_.back().s;
}

double VectormapFrenetPlugin::normalize_path_s(const double s) const
{
    if (global_samples_.empty()) {
        return s;
    }
    const double path_length = global_samples_.back().s;
    if (path_length <= EPSILON) {
        return s;
    }
    if (!route_is_loop_) {
        return std::clamp(s, 0.0, path_length);
    }
    double normalized = std::fmod(s, path_length);
    if (normalized < 0.0) {
        normalized += path_length;
    }
    return normalized;
}

uint64_t VectormapFrenetPlugin::find_nearest_lanelet(const Point2D& point) const
{
    uint64_t best_lanelet_id = 0U;
    double best_distance_sq = std::numeric_limits<double>::max();
    for (const auto& [lanelet_id, centerline_points] : lanelet_centerline_points_by_id_) {
        if (centerline_points.size() < 2U) {
            continue;
        }
        for (std::size_t i = 1U; i < centerline_points.size(); ++i) {
            const double distance_sq =
                point_segment_distance_sq(point, centerline_points[i - 1U], centerline_points[i]);
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best_lanelet_id = lanelet_id;
            }
        }
    }
    return best_lanelet_id;
}

double VectormapFrenetPlugin::adjacent_lane_offset(
    const uint64_t lanelet_id,
    const bool to_left) const
{
    const auto& adjacency =
        to_left ? left_adjacent_lanelet_by_id_ : right_adjacent_lanelet_by_id_;
    const auto adjacent_it = adjacency.find(lanelet_id);
    if (adjacent_it == adjacency.end()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const auto target_it = lanelet_centerline_points_by_id_.find(adjacent_it->second);
    if (target_it == lanelet_centerline_points_by_id_.end() || target_it->second.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double offset_sum = 0.0;
    std::size_t count = 0U;
    for (const auto& target_point : target_it->second) {
        const FrenetPoint projected = project_to_path(target_point);
        if (projected.s >= 0.0 && projected.s <= max_path_s()) {
            offset_sum += projected.d;
            ++count;
        }
    }
    if (count == 0U) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return offset_sum / static_cast<double>(count);
}

double VectormapFrenetPlugin::smooth_step(const double t)
{
    const double x = std::clamp(t, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double VectormapFrenetPlugin::yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion VectormapFrenetPlugin::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

double VectormapFrenetPlugin::point_segment_distance_sq(
    const Point2D& point,
    const Point2D& start,
    const Point2D& end)
{
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= EPSILON) {
        const double dx = point.x - start.x;
        const double dy = point.y - start.y;
        return dx * dx + dy * dy;
    }
    const double wx = point.x - start.x;
    const double wy = point.y - start.y;
    const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
    const double px = start.x + t * vx;
    const double py = start.y + t * vy;
    const double dx = point.x - px;
    const double dy = point.y - py;
    return dx * dx + dy * dy;
}

nav_msgs::msg::Path VectormapFrenetPlugin::make_path_message(
    const std::vector<PathPoint>& points,
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = path_frame_id_;
    path.poses.reserve(points.size());
    for (const auto& point : points) {
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

}  // namespace local_planner

PLUGINLIB_EXPORT_CLASS(local_planner::VectormapFrenetPlugin, local_planner::LocalPlannerPlugin)
