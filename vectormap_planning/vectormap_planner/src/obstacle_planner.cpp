#include "vectormap_planner/vectormap_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace vectormap_planner
{
namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

bool VectormapPlannerNode::find_static_obstacle(
    const double current_s,
    const double base_offset,
    const geometry_msgs::msg::PoseWithCovarianceStamped& current_pose,
    const sensor_msgs::msg::PointCloud2& pointcloud,
    double& obstacle_s,
    double& obstacle_d,
    double& avoidance_shift)
{
    if (pointcloud.width == 0U || pointcloud.height == 0U) {
        return false;
    }
    if (pointcloud.header.frame_id != map_frame_id_ && pointcloud.header.frame_id != base_frame_id_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "unsupported obstacle pointcloud frame_id: %s",
            pointcloud.header.frame_id.c_str());
        return false;
    }

    const double ego_yaw = yaw_from_quaternion(current_pose.pose.pose.orientation);
    const double cos_yaw = std::cos(ego_yaw);
    const double sin_yaw = std::sin(ego_yaw);
    const double ego_x = current_pose.pose.pose.position.x;
    const double ego_y = current_pose.pose.pose.position.y;
    const double lateral_limit =
        vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ + avoidance_soft_margin_m_ + envelope_buffer_margin_m_;

    const double current_s_normalized = normalize_path_s(current_s);
    bool found = false;
    double best_s = std::numeric_limits<double>::max();
    double d_sum = 0.0;
    std::size_t d_count = 0U;
    int skip_count = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (skip_count++ % obstacle_pointcloud_step_ != 0) {
            continue;
        }
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
            continue;
        }

        Point2D map_point{*iter_x, *iter_y};
        if (pointcloud.header.frame_id == base_frame_id_) {
            map_point.x = ego_x + cos_yaw * (*iter_x) - sin_yaw * (*iter_y);
            map_point.y = ego_y + sin_yaw * (*iter_x) + cos_yaw * (*iter_y);
        }

        const FrenetPoint frenet = project_to_path(map_point);
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
        std::min(max_avoidance_shift_m_, vehicle_width_m_ * 0.5 + avoidance_hard_margin_m_ + avoidance_soft_margin_m_);
    avoidance_shift = shift_sign * required_shift;
    return true;
}

}  // namespace vectormap_planner
