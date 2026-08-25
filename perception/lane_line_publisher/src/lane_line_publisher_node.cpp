#include "lane_line_publisher/lane_line_publisher_node.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include <camera_utility/camera_parameters.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lane_line_publisher
{

LaneLinePublisherNode::LaneLinePublisherNode(const rclcpp::NodeOptions& options)
: LaneLinePublisherNode("", options)
{
}

LaneLinePublisherNode::LaneLinePublisherNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("lane_line_publisher_node", name_space, options),
  mask_threshold_(static_cast<uint8_t>(get_parameter("mask_threshold").as_int())),
  voxel_grid_size_meter_(get_parameter("voxel_grid_size_meter").as_double()),
  ground_projection_look_up_table_(ground_projection_look_up_table(camera_utility::getCameraIntrinsics(*this), camera_utility::getBaseTCamera(*this)))
{
    mask_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/perception/lane_mask", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&LaneLinePublisherNode::lane_mask_callback, this, std::placeholders::_1));

    lane_line_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/lane_line_points", rclcpp::QoS(10));
    lane_line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/perception/lane_line", rclcpp::QoS(10));
}

void LaneLinePublisherNode::lane_mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv::Mat skeleton_mask;
    const auto mask_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat binary_mask;
    cv::threshold(mask_image->image, binary_mask, mask_threshold_, 255.0, cv::THRESH_BINARY);
    cv::ximgproc::thinning(binary_mask, skeleton_mask, cv::ximgproc::THINNING_ZHANGSUEN);

    const auto components = split_line_components(skeleton_mask, min_component_pixels_);
    std::vector<Eigen::Vector2d> observed_points;
    for (const auto& component : components) {
        const auto component_points = lane_pixels_to_base_points(component.mask, ground_projection_look_up_table_);
        observed_points.insert(observed_points.end(), component_points.begin(), component_points.end());
    }

    const auto base_points = voxel_downsample(observed_points, voxel_grid_size_meter_);

    lane_line_points_publisher_->publish(make_lane_line_point_cloud(base_points, msg->header.stamp));
    // debug用のため，subscriberがいない場合はpublishしない
    if (lane_line_marker_publisher_->get_subscription_count() > 0) {
        lane_line_marker_publisher_->publish(make_lane_line_marker_array(base_points, msg->header.stamp));
    }
}

sensor_msgs::msg::PointCloud2::UniquePtr LaneLinePublisherNode::make_lane_line_point_cloud(
    const std::vector<Eigen::Vector2d>& base_points,
    const builtin_interfaces::msg::Time& stamp) const
{
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp = stamp;
    cloud->header.frame_id = "base_link";
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(base_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
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

visualization_msgs::msg::MarkerArray::UniquePtr LaneLinePublisherNode::make_lane_line_marker_array(
    const std::vector<Eigen::Vector2d>& base_points,
    const builtin_interfaces::msg::Time& stamp) const
{
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = stamp;
    delete_marker.header.frame_id = "base_link";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(delete_marker);

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

    marker_array->markers.push_back(std::move(points_marker));
    return marker_array;
}

}
