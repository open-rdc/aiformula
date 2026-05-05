#include "object_detector/object_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace object_detector
{

ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions& options)
: ObjectDetectorNode("", options)
{
}

ObjectDetectorNode::ObjectDetectorNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("object_detector_node", name_space, options),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  ground_z_threshold_m_(get_parameter("ground_z_threshold_m").as_double()),
  voxel_leaf_size_m_(get_parameter("voxel_leaf_size_m").as_double()),
  cluster_tolerance_m_(get_parameter("cluster_tolerance_m").as_double()),
  min_cluster_size_(get_parameter("min_cluster_size").as_int()),
  max_cluster_size_(get_parameter("max_cluster_size").as_int()),
  marker_height_m_(get_parameter("marker_height_m").as_double())
{
    if (voxel_leaf_size_m_ <= 0.0 || cluster_tolerance_m_ <= 0.0 ||
        min_cluster_size_ <= 0 || max_cluster_size_ < min_cluster_size_ ||
        marker_height_m_ <= 0.0)
    {
        throw std::invalid_argument("object_detector_node: invalid parameters");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const rclcpp::QoS qos(10);
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud", qos,
        std::bind(&ObjectDetectorNode::pointcloud_callback, this, std::placeholders::_1));
    objects_publisher_ = create_publisher<object_detection_msgs::msg::ObjectInfoArray>(
        "/perception/objects", qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/perception/objects_visualize", qos);
}

void ObjectDetectorNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "[input] frame_id=%s, %u x %u = %zu pts",
        msg->header.frame_id.c_str(), msg->width, msg->height,
        static_cast<std::size_t>(msg->width) * msg->height);

    // Step 1: Transform PointCloud2 from camera frame to base_link frame
    geometry_msgs::msg::TransformStamped tf_to_base;
    try {
        tf_to_base = tf_buffer_->lookupTransform(
            base_frame_id_, msg->header.frame_id, rclcpp::Time(0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            get_logger(),
            "TF lookup failed (%s -> %s): %s",
            msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_base_msg;
    tf2::doTransform(*msg, cloud_base_msg, tf_to_base);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_base_msg, *cloud);

    if (cloud->empty()) {
        RCLCPP_WARN(get_logger(), "cloud empty after transform to base_link");
        publish_empty(msg->header.stamp);
        return;
    }

    // Ground filter: keep points above ground_z_threshold_m in base_link frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(
            static_cast<float>(ground_z_threshold_m_),
            std::numeric_limits<float>::max());
        pass.filter(*filtered);
    }

    RCLCPP_DEBUG(
        get_logger(),
        "[passthrough z>%.3f] %zu -> %zu pts",
        ground_z_threshold_m_, cloud->size(), filtered->size());

    if (filtered->empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "all points removed by ground filter (z>%.3f m) — adjust ground_z_threshold_m",
            ground_z_threshold_m_);
        publish_empty(msg->header.stamp);
        return;
    }

    // VoxelGrid downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(filtered);
        const auto leaf = static_cast<float>(voxel_leaf_size_m_);
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*voxeled);
    }

    RCLCPP_DEBUG(
        get_logger(),
        "[voxelgrid leaf=%.3f] %zu -> %zu pts",
        voxel_leaf_size_m_, filtered->size(), voxeled->size());

    if (voxeled->empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "voxelgrid result empty");
        publish_empty(msg->header.stamp);
        return;
    }

    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(voxeled);
    std::vector<pcl::PointIndices> cluster_indices;
    {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(static_cast<double>(cluster_tolerance_m_));
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(voxeled);
        ec.extract(cluster_indices);
    }

    RCLCPP_DEBUG(
        get_logger(),
        "[euclidean tol=%.3f min=%d max=%d] %zu clusters from %zu pts",
        cluster_tolerance_m_, min_cluster_size_, max_cluster_size_,
        cluster_indices.size(), voxeled->size());

    if (cluster_indices.empty()) {
        RCLCPP_DEBUG(
            get_logger(),
            "[euclidean] no clusters from %zu pts — adjust tolerance/min_cluster_size",
            voxeled->size());
        publish_empty(msg->header.stamp);
        return;
    }

    // Step 2: Lookup TF for centroid transform: base_link -> map
    geometry_msgs::msg::TransformStamped tf_to_map;
    try {
        tf_to_map = tf_buffer_->lookupTransform(
            map_frame_id_, base_frame_id_, rclcpp::Time(0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
            get_logger(),
            "TF lookup failed (%s -> %s): %s",
            base_frame_id_.c_str(), map_frame_id_.c_str(), ex.what());
        return;
    }

    object_detection_msgs::msg::ObjectInfoArray objects_msg;
    objects_msg.header.stamp = msg->header.stamp;
    objects_msg.header.frame_id = map_frame_id_;

    visualization_msgs::msg::MarkerArray marker_array;

    uint8_t id = 0U;
    for (const auto& indices : cluster_indices) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        for (const int idx : indices.indices) {
            const auto& pt = voxeled->points[static_cast<std::size_t>(idx)];
            sum_x += pt.x;
            sum_y += pt.y;
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }
        const double count = static_cast<double>(indices.indices.size());
        const float width = std::max(max_x - min_x, max_y - min_y);

        // Transform centroid from base_link to map frame
        geometry_msgs::msg::PointStamped centroid_base;
        centroid_base.header.stamp = msg->header.stamp;
        centroid_base.header.frame_id = base_frame_id_;
        centroid_base.point.x = sum_x / count;
        centroid_base.point.y = sum_y / count;
        centroid_base.point.z = 0.0;

        geometry_msgs::msg::PointStamped centroid_map;
        tf2::doTransform(centroid_base, centroid_map, tf_to_map);

        const float cx = static_cast<float>(centroid_map.point.x);
        const float cy = static_cast<float>(centroid_map.point.y);

        object_detection_msgs::msg::ObjectInfo obj;
        obj.id = id;
        obj.x = cx;
        obj.y = cy;
        obj.width = width;
        objects_msg.objects.push_back(obj);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = msg->header.stamp;
        marker.header.frame_id = map_frame_id_;
        marker.ns = "objects";
        marker.id = static_cast<int32_t>(id);
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cx;
        marker.pose.position.y = cy;
        marker.pose.position.z = marker_height_m_ * 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::max(width, 0.1f);
        marker.scale.y = std::max(width, 0.1f);
        marker.scale.z = marker_height_m_;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.3);
        marker_array.markers.push_back(marker);

        ++id;
    }

    objects_publisher_->publish(objects_msg);
    marker_publisher_->publish(marker_array);
}

void ObjectDetectorNode::publish_empty(const rclcpp::Time& stamp)
{
    object_detection_msgs::msg::ObjectInfoArray objects_msg;
    objects_msg.header.stamp = stamp;
    objects_msg.header.frame_id = map_frame_id_;
    objects_publisher_->publish(objects_msg);

    visualization_msgs::msg::MarkerArray marker_array;
    marker_publisher_->publish(marker_array);
}

}  // namespace object_detector
