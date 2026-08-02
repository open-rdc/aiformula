#include "local_planner/plugins/frenet_planner_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "local_planner/frenet/frenet_planner.hpp"
#include "local_planner/frenet/hard_constraint.hpp"
#include "utilities/utils.hpp"

namespace local_planner
{
namespace
{
constexpr double EPSILON = 1.0e-6;
constexpr double AVOIDANCE_STEERING_SAFETY_FACTOR = 0.8;
constexpr double COLLISION_LONGITUDINAL_MIN_M = 0.5;
constexpr double MAX_HEADING_ERROR_RAD = 0.25 * utils::d_pi;
}

void FrenetPlannerPlugin::initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    logger_ = logger;
    clock_ = clock;

    local_path_horizon_m_ = params->get_parameter("local_path_horizon_m").get_value<double>();
    local_path_resample_interval_m_ = params->get_parameter("local_path_resample_interval_m").get_value<double>();
    max_centerline_connection_gap_m_ = params->get_parameter("max_centerline_connection_gap_m").get_value<double>();
    vehicle_width_m_ = params->get_parameter("vehicle_width_m").get_value<double>();
    avoidance_detection_forward_distance_m_ = params->get_parameter("avoidance_detection_forward_distance_m").get_value<double>();
    avoidance_hard_margin_m_ = params->get_parameter("avoidance_hard_margin_m").get_value<double>();
    avoidance_soft_margin_m_ = params->get_parameter("avoidance_soft_margin_m").get_value<double>();
    envelope_buffer_margin_m_ = params->get_parameter("envelope_buffer_margin_m").get_value<double>();
    max_avoidance_shift_m_ = params->get_parameter("max_avoidance_shift_m").get_value<double>();
    frenet_lateral_sample_step_m_ = params->get_parameter("frenet.lateral_sample_step_m").get_value<double>();
    frenet_collision_check_margin_m_ = params->get_parameter("frenet.collision_check_margin_m").get_value<double>();
    frenet_target_lengths_m_ = params->get_parameter("frenet.target_lengths_m").get_value<std::vector<double>>();
    cost_weights_.curvature = params->get_parameter("frenet.weight_curvature").get_value<double>();
    cost_weights_.length = params->get_parameter("frenet.weight_length").get_value<double>();
    cost_weights_.lateral_deviation = params->get_parameter("frenet.weight_lateral_deviation").get_value<double>();
    stop_standoff_m_ = params->get_parameter("stop_standoff_m").get_value<double>();

    const double wheelbase_m = params->get_parameter("wheelbase").get_value<double>();
    const double steering_max_deg = params->get_parameter("steering_max.pos").get_value<double>();
    const double steering_max_rad = utils::dtor(steering_max_deg);

    kappa_max_ = AVOIDANCE_STEERING_SAFETY_FACTOR * std::tan(steering_max_rad) / wheelbase_m;
}

void FrenetPlannerPlugin::setGlobalPath(const nav_msgs::msg::Path & global_path)
{
    if (global_path.poses.size() < 2U) {
        global_path_ready_ = false;
        RCLCPP_WARN(logger_, "global_pathの点数が2未満のため無視する");
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
        const double yaw = utils::yaw_from_quaternion(pose.pose.orientation);
        global_samples_.push_back(PathPoint{s, pose.pose.position.x, pose.pose.position.y, yaw});
    }

    const auto& first = global_samples_.front();
    const auto& last = global_samples_.back();
    route_is_loop_ =
        std::hypot(last.x - first.x, last.y - first.y) < max_centerline_connection_gap_m_;
    global_path_ready_ = true;
}

std::optional<nav_msgs::msg::Path> FrenetPlannerPlugin::computeLocalPath(
    const geometry_msgs::msg::PoseWithCovarianceStamped & ego_pose,
    const geometry_msgs::msg::TwistWithCovarianceStamped &,
    const object_detection_msgs::msg::ObjectInfoArray * objects)
{
    if (!global_path_ready_) {
        return std::nullopt;
    }

    const auto& position = ego_pose.pose.pose.position;
    const ProjectedPose ego = project_to_path(Point2D{position.x, position.y});
    const double ego_yaw = utils::yaw_from_quaternion(ego_pose.pose.pose.orientation);
    const double heading_error = std::clamp(
        std::remainder(ego_yaw - ego.path_yaw, 2.0 * utils::d_pi),
        -MAX_HEADING_ERROR_RAD, MAX_HEADING_ERROR_RAD);
    const double reference_curvature = reference_curvature_at(ego.s);
    const frenet::FrenetState initial{
        ego.s, ego.d,
        (1.0 - reference_curvature * ego.d) * std::tan(heading_error),
        0.0};

    std::optional<FrenetObstacle> obstacle;
    if (objects && !objects->objects.empty()) {
        obstacle = find_static_obstacle(ego.s, *objects);
    }

    const double end_s = route_is_loop_ ?
        ego.s + local_path_horizon_m_ :
        std::min(ego.s + local_path_horizon_m_, max_path_s());
    if (end_s <= ego.s + EPSILON) {
        return std::nullopt;
    }

    auto points = plan_best_path(ego.s, end_s, initial, obstacle);
    if (points.empty() && obstacle) {
        points = make_stop_path(initial, *obstacle);
    }
    if (points.empty()) {
        return std::nullopt;
    }
    return make_path_message(points, clock_->now());
}

std::vector<FrenetPlannerPlugin::PathPoint> FrenetPlannerPlugin::plan_best_path(
    const double start_s,
    const double end_s,
    const frenet::FrenetState& initial,
    const std::optional<FrenetObstacle>& obstacle) const
{
    const auto target_s_list = make_target_s_list(start_s, end_s);
    const auto target_d_list = make_target_grid(obstacle);
    const double collision_lateral_margin = obstacle ?
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ +
        envelope_buffer_margin_m_ + frenet_collision_check_margin_m_ +
        obstacle->half_width :
        0.0;
    const double collision_longitudinal_margin =
        std::max(COLLISION_LONGITUDINAL_MIN_M, local_path_resample_interval_m_ * 2.0);

    struct Candidate
    {
        double target_d;
        std::vector<PathPoint> points;
        bool curvature_ok;
        double cost;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(target_s_list.size() * target_d_list.size());
    for (const double target_s : target_s_list) {
        for (const double target_d : target_d_list) {
            const auto frenet_candidate = frenet::generate_candidate(
                initial, frenet::FrenetState{target_s, target_d, 0.0, 0.0},
                local_path_resample_interval_m_);
            if (frenet_candidate.s_points.size() < 2U) {
                continue;
            }
            if (obstacle && !frenet::is_collision_free(
                    frenet_candidate.s_points, frenet_candidate.offsets,
                    obstacle->s, obstacle->d,
                    collision_lateral_margin, collision_longitudinal_margin)) {
                continue;
            }
            auto points = to_cartesian(
                sample_reference(frenet_candidate.s_points), frenet_candidate.offsets);
            const auto curvatures = compute_curvatures(points);
            const bool curvature_ok =
                frenet::satisfies_curvature_limit(curvatures, kappa_max_);
            const double cost = frenet::candidate_cost(
                curvatures, compute_path_length(points),
                frenet_candidate.offsets.back(), cost_weights_);
            candidates.push_back(
                Candidate{target_d, std::move(points), curvature_ok, cost});
        }
    }
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });

    for (auto& candidate : candidates) {
        if (candidate.curvature_ok) {
            return std::move(candidate.points);
        }
    }
    for (auto& candidate : candidates) {
        if (candidate.target_d == 0.0) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "回避シフトなしの基本経路の曲率がkappa_max(%.3f)を超過している", kappa_max_);
            return std::move(candidate.points);
        }
    }
    return {};
}

std::vector<FrenetPlannerPlugin::PathPoint> FrenetPlannerPlugin::make_stop_path(
    const frenet::FrenetState& initial,
    const FrenetObstacle& obstacle) const
{
    const double stop_end_s = std::max(
        initial.s + local_path_resample_interval_m_,
        obstacle.s - stop_standoff_m_);
    const auto candidate = frenet::generate_candidate(
        initial, frenet::FrenetState{stop_end_s, 0.0, 0.0, 0.0},
        local_path_resample_interval_m_);
    return to_cartesian(sample_reference(candidate.s_points), candidate.offsets);
}

std::vector<double> FrenetPlannerPlugin::make_target_s_list(
    const double start_s, const double end_s) const
{
    std::vector<double> targets;
    targets.reserve(frenet_target_lengths_m_.size());
    for (const double length : frenet_target_lengths_m_) {
        const double target_s = std::min(end_s, start_s + length);
        targets.push_back(target_s);
        if (target_s >= end_s) {
            break;
        }
    }
    if (targets.empty()) {
        targets.push_back(end_s);
    }
    return targets;
}

std::vector<double> FrenetPlannerPlugin::make_target_grid(
    const std::optional<FrenetObstacle>& obstacle) const
{
    const double step = frenet_lateral_sample_step_m_ > EPSILON ?
        frenet_lateral_sample_step_m_ : max_avoidance_shift_m_;
    const int step_count = std::max(
        1, static_cast<int>(std::ceil(max_avoidance_shift_m_ / step - EPSILON)));
    const double away_sign = (obstacle && obstacle->d >= 0.0) ? -1.0 : 1.0;

    std::vector<double> targets;
    targets.reserve(2U * static_cast<std::size_t>(step_count) + 1U);
    targets.push_back(0.0);
    for (int k = 1; k <= step_count; ++k) {
        const double magnitude = std::min(k * step, max_avoidance_shift_m_);
        targets.push_back(away_sign * magnitude);
        targets.push_back(-away_sign * magnitude);
    }
    return targets;
}

std::optional<FrenetPlannerPlugin::FrenetObstacle> FrenetPlannerPlugin::find_static_obstacle(
    const double current_s,
    const object_detection_msgs::msg::ObjectInfoArray& objects) const
{
    const double lateral_limit =
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ +
        avoidance_soft_margin_m_ + envelope_buffer_margin_m_;
    const double current_s_normalized = normalize_path_s(current_s);

    std::optional<FrenetObstacle> nearest;
    double nearest_delta_s = std::numeric_limits<double>::max();
    for (const auto& obj : objects.objects) {
        const ProjectedPose frenet_pose = project_to_path(Point2D{obj.x, obj.y});
        const double half_width = 0.5 * std::max(0.0, static_cast<double>(obj.width));
        double delta_s = frenet_pose.s - current_s_normalized;
        if (route_is_loop_ && delta_s < 0.0) {
            delta_s += max_path_s();
        }
        if (delta_s < 0.0 || delta_s > avoidance_detection_forward_distance_m_) {
            continue;
        }
        if (std::abs(frenet_pose.d) > lateral_limit + half_width) {
            continue;
        }
        if (delta_s < nearest_delta_s) {
            nearest_delta_s = delta_s;
            nearest = FrenetObstacle{current_s + delta_s, frenet_pose.d, half_width};
        }
    }
    return nearest;
}

std::vector<FrenetPlannerPlugin::PathPoint> FrenetPlannerPlugin::sample_reference(
    const std::vector<double>& s_grid) const
{
    std::vector<PathPoint> reference;
    reference.reserve(s_grid.size());
    for (const double s : s_grid) {
        reference.push_back(path_point_at_s(s));
    }
    return reference;
}

std::vector<FrenetPlannerPlugin::PathPoint> FrenetPlannerPlugin::to_cartesian(
    const std::vector<PathPoint>& reference,
    const std::vector<double>& offsets) const
{
    std::vector<PathPoint> points;
    const std::size_t size = std::min(reference.size(), offsets.size());
    points.reserve(size);
    for (std::size_t i = 0U; i < size; ++i) {
        const auto& base = reference[i];
        points.push_back(PathPoint{
            base.s,
            base.x - std::sin(base.yaw) * offsets[i],
            base.y + std::cos(base.yaw) * offsets[i],
            base.yaw});
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

double FrenetPlannerPlugin::reference_curvature_at(const double s) const
{
    const double h = local_path_resample_interval_m_;
    const auto behind = path_point_at_s(s - h);
    const auto ahead = path_point_at_s(s + h);
    const double dyaw = std::remainder(ahead.yaw - behind.yaw, 2.0 * utils::d_pi);
    return dyaw / (2.0 * h);
}

std::vector<double> FrenetPlannerPlugin::compute_curvatures(
    const std::vector<PathPoint>& points)
{
    std::vector<double> curvatures;
    if (points.size() < 3U) {
        return curvatures;
    }
    curvatures.reserve(points.size() - 2U);
    for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
        const double segment = std::hypot(
            points[i].x - points[i - 1U].x,
            points[i].y - points[i - 1U].y);
        if (segment < EPSILON) {
            curvatures.push_back(0.0);
            continue;
        }
        const double dyaw = std::remainder(
            points[i].yaw - points[i - 1U].yaw, 2.0 * utils::d_pi);
        curvatures.push_back(dyaw / segment);
    }
    return curvatures;
}

double FrenetPlannerPlugin::compute_path_length(const std::vector<PathPoint>& points)
{
    double length = 0.0;
    for (std::size_t i = 1U; i < points.size(); ++i) {
        length += std::hypot(
            points[i].x - points[i - 1U].x,
            points[i].y - points[i - 1U].y);
    }
    return length;
}

FrenetPlannerPlugin::ProjectedPose FrenetPlannerPlugin::project_to_path(
    const Point2D& point) const
{
    double best_distance_sq = std::numeric_limits<double>::max();
    ProjectedPose best{global_samples_.front().s, 0.0, global_samples_.front().yaw};
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
            best.path_yaw = yaw;
        }
    }
    return best;
}

FrenetPlannerPlugin::PathPoint FrenetPlannerPlugin::path_point_at_s(
    const double s) const
{
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
        std::atan2(end.y - start.y, end.x - start.x)};
}

double FrenetPlannerPlugin::max_path_s() const
{
    return global_samples_.back().s;
}

double FrenetPlannerPlugin::normalize_path_s(const double s) const
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

nav_msgs::msg::Path FrenetPlannerPlugin::make_path_message(
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
        pose.pose.orientation = utils::yaw_to_quaternion(point.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

}

PLUGINLIB_EXPORT_CLASS(local_planner::FrenetPlannerPlugin, local_planner::LocalPlannerPlugin)
