#include "lane_line_publisher/lane_line_publisher_node.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <utility>

#include <camera_utility/camera_parameters.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lane_line_publisher
{
namespace
{

GroundProjectionLUT make_ground_projection_lut(
    rclcpp::Node& node,
    const camera_utility::CameraIntrinsics& intrinsics)
{
    return build_ground_projection_lut(
        intrinsics,
        camera_utility::getBaseTCamera(node),
        node.get_parameter("ground_plane_z_base").as_double(),
        node.get_parameter("max_ground_intersection_distance").as_double());
}

std::vector<Eigen::Vector2d> voxel_downsample(
    const std::vector<Eigen::Vector2d>& points,
    const double voxel_size)
{
    struct Voxel
    {
        Eigen::Vector2d sum = Eigen::Vector2d::Zero();
        std::size_t count = 0U;
    };

    std::map<std::pair<std::int64_t, std::int64_t>, Voxel> voxels;
    for (const auto& point : points) {
        if (!point.allFinite()) {
            continue;
        }
        const auto x_index = static_cast<std::int64_t>(std::floor(point.x() / voxel_size));
        const auto y_index = static_cast<std::int64_t>(std::floor(point.y() / voxel_size));
        auto& voxel = voxels[{x_index, y_index}];
        voxel.sum += point;
        ++voxel.count;
    }

    std::vector<Eigen::Vector2d> downsampled_points;
    downsampled_points.reserve(voxels.size());
    for (const auto& [index, voxel] : voxels) {
        (void)index;
        downsampled_points.push_back(voxel.sum / static_cast<double>(voxel.count));
    }
    return downsampled_points;
}

}  // namespace

LaneLinePublisherNode::LaneLinePublisherNode(const rclcpp::NodeOptions& options)
: LaneLinePublisherNode("", options)
{
}

LaneLinePublisherNode::LaneLinePublisherNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("lane_line_publisher_node", name_space, options),
  mask_threshold_(static_cast<uint8_t>(get_parameter("mask_threshold").as_int())),
  max_observed_points_(static_cast<std::size_t>(get_parameter("max_observed_points").as_int())),
  voxel_size_m_(get_parameter("voxel_size_m").as_double()),
  max_point_link_distance_m_(get_parameter("max_point_link_distance_m").as_double()),
  point_resample_interval_m_(get_parameter("point_resample_interval_m").as_double()),
  camera_intrinsics_(camera_utility::getCameraIntrinsics(*this)),
  ground_projection_lut_(make_ground_projection_lut(*this, camera_intrinsics_))
{
    if (!(voxel_size_m_ > 0.0) || !std::isfinite(voxel_size_m_)) {
        throw std::invalid_argument("voxel_size_m must be greater than 0");
    }
    if (!(max_point_link_distance_m_ > 0.0) || !std::isfinite(max_point_link_distance_m_)) {
        throw std::invalid_argument("max_point_link_distance_m must be greater than 0");
    }
    if (!(point_resample_interval_m_ > 0.0) || !std::isfinite(point_resample_interval_m_)) {
        throw std::invalid_argument("point_resample_interval_m must be greater than 0");
    }

    mask_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/perception/lane_mask", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&LaneLinePublisherNode::lane_mask_callback, this, std::placeholders::_1));

    lane_line_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/lane_line_points", rclcpp::QoS(10));
    lane_line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/perception/lane_line", rclcpp::QoS(10));
}

void LaneLinePublisherNode::lane_mask_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat skeleton_mask;
    try {
        const auto mask_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat binary_mask;
        cv::threshold(mask_image->image, binary_mask, mask_threshold_, 255.0, cv::THRESH_BINARY);
        cv::ximgproc::thinning(binary_mask, skeleton_mask, cv::ximgproc::THINNING_ZHANGSUEN);
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "lane maskの変換に失敗したためスキップする: %s", error.what());
        return;
    }

    if (skeleton_mask.cols != camera_intrinsics_.width ||
        skeleton_mask.rows != camera_intrinsics_.height)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "lane maskのサイズがcamera.sizeの解像度と一致しないためスキップする");
        return;
    }

    const auto downsampled_points = voxel_downsample(
        lane_pixels_to_base_points(skeleton_mask, ground_projection_lut_, max_observed_points_),
        voxel_size_m_);
    const auto base_points = resample_lane_points(
        downsampled_points, max_point_link_distance_m_, point_resample_interval_m_);

    lane_line_points_publisher_->publish(make_lane_line_point_cloud(base_points, msg->header.stamp));
    lane_line_marker_publisher_->publish(make_lane_line_marker_array(base_points, msg->header.stamp));
}

sensor_msgs::msg::PointCloud2 LaneLinePublisherNode::make_lane_line_point_cloud(
    const std::vector<Eigen::Vector2d>& base_points,
    const builtin_interfaces::msg::Time& stamp) const
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(base_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto& p : base_points) {
        *iter_x = static_cast<float>(p.x());
        *iter_y = static_cast<float>(p.y());
        *iter_z = 0.0F;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    return cloud;
}

visualization_msgs::msg::MarkerArray LaneLinePublisherNode::make_lane_line_marker_array(
    const std::vector<Eigen::Vector2d>& base_points,
    const builtin_interfaces::msg::Time& stamp) const
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = stamp;
    delete_marker.header.frame_id = "base_link";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker points_marker;
    points_marker.header = delete_marker.header;
    points_marker.ns = "lane_line";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::POINTS;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.08;
    points_marker.scale.y = 0.08;
    points_marker.color.r = 0.0F;
    points_marker.color.g = 1.0F;
    points_marker.color.b = 0.2F;
    points_marker.color.a = 1.0F;
    points_marker.points.reserve(base_points.size());

    for (const auto& p : base_points) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        points_marker.points.push_back(point);
    }

    marker_array.markers.push_back(std::move(points_marker));
    return marker_array;
}

}
