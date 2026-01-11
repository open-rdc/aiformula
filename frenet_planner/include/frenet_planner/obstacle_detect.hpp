#pragma once

#include <vector>
#include <unordered_map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/time.hpp>

namespace frenet_planner {

struct Obstacle {
    double x;
    double y;
    double s;
    double d;
};

class ObstacleDetector {
public:
    ObstacleDetector();

    std::vector<Obstacle> detect_obstacles(
        const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud
    );

    visualization_msgs::msg::MarkerArray obstacles_to_marker_array(
        const std::vector<Obstacle>& obstacles,
        const std::string& frame_id,
        const rclcpp::Time& stamp
    ) const;

private:
    double z_min_;
    double z_max_;
    double voxel_size_;

    struct VoxelKey {
        int x, y, z;

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& key) const {
            std::size_t h1 = std::hash<int>{}(key.x);
            std::size_t h2 = std::hash<int>{}(key.y);
            std::size_t h3 = std::hash<int>{}(key.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    VoxelKey get_voxel_key(double x, double y, double z) const;
};

}  // namespace frenet_planner
