#include "frenet_planner/obstacle_detect.hpp"
#include <cmath>

namespace frenet_planner {

ObstacleDetector::ObstacleDetector()
    : voxel_size_(0.3) {}

ObstacleDetector::VoxelKey ObstacleDetector::get_voxel_key(double x, double y, double z) const {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(x / voxel_size_));
    key.y = static_cast<int>(std::floor(y / voxel_size_));
    key.z = static_cast<int>(std::floor(z / voxel_size_));
    return key;
}

Obstacle ObstacleDetector::voxel_center(const VoxelKey& key) const {
    Obstacle obs;
    obs.x = (static_cast<double>(key.x) + 0.5) * voxel_size_;
    obs.y = (static_cast<double>(key.y) + 0.5) * voxel_size_;
    obs.s = 0.0;
    obs.d = 0.0;
    return obs;
}

std::vector<Obstacle> ObstacleDetector::detect_obstacles(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud
) {
    if (!pointcloud || pointcloud->data.empty()) {
        return {};
    }

    std::vector<Obstacle> obstacles;
    std::unordered_set<VoxelKey, VoxelKeyHash> voxel_set;
    size_t point_count = static_cast<size_t>(pointcloud->width) * static_cast<size_t>(pointcloud->height);
    if (point_count == 0 && pointcloud->point_step > 0) {
        point_count = pointcloud->data.size() / pointcloud->point_step;
    }
    voxel_set.reserve(point_count);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double obs_x = *iter_x;
        double obs_y = *iter_y;
        double obs_z = *iter_z;

        VoxelKey voxel_key = get_voxel_key(obs_x, obs_y, obs_z);
        voxel_set.insert(voxel_key);
    }

    obstacles.reserve(voxel_set.size());
    for (const auto& key : voxel_set) {
        obstacles.push_back(voxel_center(key));
    }

    return obstacles;
}

visualization_msgs::msg::MarkerArray ObstacleDetector::obstacles_to_marker_array(
    const std::vector<Obstacle>& obstacles,
    const std::string& frame_id,
    const rclcpp::Time& stamp
) const {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "obstacles";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < obstacles.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "obstacles";
        marker.id = i + 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = obstacles[i].x;
        marker.pose.position.y = obstacles[i].y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = voxel_size_ * 0.8;
        marker.scale.y = voxel_size_ * 0.8;
        marker.scale.z = voxel_size_ * 0.8;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

}  // namespace frenet_planner
