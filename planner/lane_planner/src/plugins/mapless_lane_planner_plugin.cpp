#include "lane_planner/mapless_lane_planner_plugin.hpp"

#include <cmath>
#include <stdexcept>

namespace lane_planner
{
namespace
{

constexpr double EPSILON = 1.0e-9;

// 3x3 rotation matrix rows from quaternion (x, y, z, w)
struct RotMat3
{
    double r[3][3];
};

RotMat3 quat_to_rotmat(double qx, double qy, double qz, double qw)
{
    RotMat3 R{};
    R.r[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    R.r[0][1] = 2.0 * (qx * qy - qz * qw);
    R.r[0][2] = 2.0 * (qx * qz + qy * qw);
    R.r[1][0] = 2.0 * (qx * qy + qz * qw);
    R.r[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
    R.r[1][2] = 2.0 * (qy * qz - qx * qw);
    R.r[2][0] = 2.0 * (qx * qz - qy * qw);
    R.r[2][1] = 2.0 * (qy * qz + qx * qw);
    R.r[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
    return R;
}

geometry_msgs::msg::Point transform_point(
    const geometry_msgs::msg::Point & p_base,
    const geometry_msgs::msg::Pose & ego_in_map)
{
    const auto & o = ego_in_map.orientation;
    const auto R = quat_to_rotmat(o.x, o.y, o.z, o.w);
    geometry_msgs::msg::Point p_map;
    p_map.x = R.r[0][0] * p_base.x + R.r[0][1] * p_base.y + R.r[0][2] * p_base.z
              + ego_in_map.position.x;
    p_map.y = R.r[1][0] * p_base.x + R.r[1][1] * p_base.y + R.r[1][2] * p_base.z
              + ego_in_map.position.y;
    p_map.z = R.r[2][0] * p_base.x + R.r[2][1] * p_base.y + R.r[2][2] * p_base.z
              + ego_in_map.position.z;
    return p_map;
}

}  // namespace

// ---------- LanePlannerPlugin interface ----------

void MaplessLanePlannerPlugin::initialize(rclcpp::Node & node)
{
    clock_ = node.get_clock();
    logger_ = node.get_logger();

    map_frame_id_ = node.get_parameter("map_frame_id").as_string();
    max_corridor_length_m_ = node.get_parameter("max_corridor_length_m").as_double();
    path_resample_interval_m_ = node.get_parameter("path_resample_interval_m").as_double();

    if (max_corridor_length_m_ <= 0.0) {
        throw std::invalid_argument("max_corridor_length_m must be greater than 0");
    }
    if (path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("path_resample_interval_m must be greater than 0");
    }

    RCLCPP_INFO(logger_, "MaplessLanePlannerPlugin initialized");
}

void MaplessLanePlannerPlugin::set_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(pose);
}

void MaplessLanePlannerPlugin::set_nav_command(const std::string & command)
{
    int16_t target = mapless_planning_msgs::msg::MissionLanesStamped::LANE_KEEP;
    if (command == "left") {
        target = mapless_planning_msgs::msg::MissionLanesStamped::LANE_CHANGE_LEFT;
    } else if (command == "right") {
        target = mapless_planning_msgs::msg::MissionLanesStamped::LANE_CHANGE_RIGHT;
    } else if (command != "straight" && command != "keep") {
        RCLCPP_ERROR(logger_, "unknown nav_cmd for mapless: '%s'", command.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_lane_ = static_cast<int>(target);
}

void MaplessLanePlannerPlugin::set_road_segments(
    const mapless_planning_msgs::msg::RoadSegments & segments)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_segments_ = segments;
}

std::optional<nav_msgs::msg::Path> MaplessLanePlannerPlugin::compute_path(
    const rclcpp::Time & stamp)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_segments_) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000, "no RoadSegments received yet");
        return std::nullopt;
    }
    if (!latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000, "no localization pose received yet");
        return std::nullopt;
    }

    if (latest_segments_->segments.empty()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "RoadSegments has no segments");
        return std::nullopt;
    }

    if (target_lane_ != mapless_planning_msgs::msg::MissionLanesStamped::LANE_KEEP) {
        RCLCPP_ERROR_THROTTLE(
            logger_, *clock_, 2000,
            "lane change requested (target_lane=%d) but not implemented in initial version; "
            "no path published",
            target_lane_);
        return std::nullopt;
    }

    const auto ego_corridor = build_ego_corridor(*latest_segments_);

    // Build MissionLanesStamped for external use
    mapless_planning_msgs::msg::MissionLanesStamped ml;
    ml.header.stamp = stamp;
    ml.header.frame_id = latest_segments_->header.frame_id;
    ml.ego_lane = ego_corridor;
    ml.lane_with_goal_point = ego_corridor;
    ml.target_lane = static_cast<int16_t>(target_lane_);
    latest_mission_lanes_ = ml;

    if (ego_corridor.centerline.empty()) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000, "ego corridor centerline is empty");
        return std::nullopt;
    }

    return corridor_to_path(ego_corridor, latest_pose_->pose.pose, stamp);
}

std::optional<mapless_planning_msgs::msg::MissionLanesStamped>
MaplessLanePlannerPlugin::get_mission_lanes() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_mission_lanes_;
}

// ---------- Internal helpers ----------

mapless_planning_msgs::msg::DrivingCorridor MaplessLanePlannerPlugin::build_ego_corridor(
    const mapless_planning_msgs::msg::RoadSegments & segments) const
{
    // Initial version: use segment 0 as ego lane
    const auto & seg = segments.segments[0];

    if (seg.linestrings.size() < 2 ||
        seg.linestrings[0].poses.empty() ||
        seg.linestrings[1].poses.empty())
    {
        RCLCPP_WARN(logger_, "segment 0 has insufficient linestrings");
        return mapless_planning_msgs::msg::DrivingCorridor{};
    }

    const auto & left_ls = seg.linestrings[0];
    const auto & right_ls = seg.linestrings[1];

    mapless_planning_msgs::msg::DrivingCorridor corridor;

    for (const auto & pose : left_ls.poses) {
        corridor.bound_left.push_back(pose.position);
    }
    for (const auto & pose : right_ls.poses) {
        corridor.bound_right.push_back(pose.position);
    }

    // Compute centerline: interpolate between left and right at same x samples
    const std::size_t n_left = left_ls.poses.size();
    const std::size_t n_right = right_ls.poses.size();
    const std::size_t n_pts = std::max(n_left, n_right);

    for (std::size_t i = 0; i < n_pts; ++i) {
        const double tl = static_cast<double>(i) / std::max<std::size_t>(1, n_pts - 1);
        const double tr = tl;

        const auto idx_l = static_cast<std::size_t>(tl * static_cast<double>(n_left - 1));
        const auto idx_r = static_cast<std::size_t>(tr * static_cast<double>(n_right - 1));

        geometry_msgs::msg::Point c;
        c.x = 0.5 * (left_ls.poses[idx_l].position.x + right_ls.poses[idx_r].position.x);
        c.y = 0.5 * (left_ls.poses[idx_l].position.y + right_ls.poses[idx_r].position.y);
        c.z = 0.0;
        corridor.centerline.push_back(c);
    }

    return corridor;
}

nav_msgs::msg::Path MaplessLanePlannerPlugin::corridor_to_path(
    const mapless_planning_msgs::msg::DrivingCorridor & corridor,
    const geometry_msgs::msg::Pose & ego_in_map,
    const rclcpp::Time & stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_id_;

    const auto & cl = corridor.centerline;
    if (cl.size() < 2) {
        return path;
    }

    path.poses.reserve(cl.size());
    for (std::size_t i = 0; i < cl.size(); ++i) {
        const auto p_map = transform_point(cl[i], ego_in_map);

        double yaw = 0.0;
        if (i + 1 < cl.size()) {
            const auto p_next = transform_point(cl[i + 1], ego_in_map);
            const double dx = p_next.x - p_map.x;
            const double dy = p_next.y - p_map.y;
            if (std::hypot(dx, dy) > EPSILON) {
                yaw = std::atan2(dy, dx);
            }
        } else if (i > 0) {
            const auto p_prev = transform_point(cl[i - 1], ego_in_map);
            const double dx = p_map.x - p_prev.x;
            const double dy = p_map.y - p_prev.y;
            if (std::hypot(dx, dy) > EPSILON) {
                yaw = std::atan2(dy, dx);
            }
        }

        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position = p_map;
        ps.pose.orientation = yaw_to_quaternion(yaw);
        path.poses.push_back(ps);
    }

    return path;
}

geometry_msgs::msg::Quaternion MaplessLanePlannerPlugin::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace lane_planner
