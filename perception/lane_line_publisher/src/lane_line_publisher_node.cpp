#include "lane_line_publisher/lane_line_publisher_node.hpp"

#include <functional>
#include <utility>

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

CameraModel make_camera_model(rclcpp::Node& node)
{
    CameraModel camera_model;
    camera_model.fx = node.get_parameter("camera.fx").as_double();
    camera_model.fy = node.get_parameter("camera.fy").as_double();
    camera_model.cx = node.get_parameter("camera.cx").as_double();
    camera_model.cy = node.get_parameter("camera.cy").as_double();
    camera_model.ground_plane_z_base = node.get_parameter("ground_plane_z_base").as_double();
    camera_model.min_ground_intersection_distance = node.get_parameter("min_ground_intersection_distance").as_double();
    camera_model.max_ground_intersection_distance = node.get_parameter("max_ground_intersection_distance").as_double();

    const double roll = node.get_parameter("camera_to_base.roll").as_double();
    const double pitch = node.get_parameter("camera_to_base.pitch").as_double();
    const double yaw = node.get_parameter("camera_to_base.yaw").as_double();

    camera_model.camera_to_base_rotation = rotation_matrix_from_rpy(roll, pitch, yaw);
    camera_model.camera_to_base_translation = Eigen::Vector3d(
        node.get_parameter("camera_to_base.x").as_double(),
        node.get_parameter("camera_to_base.y").as_double(),
        node.get_parameter("camera_to_base.z").as_double());

    return camera_model;
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
  pixel_step_(get_parameter("pixel_step").as_int()),
  max_observed_points_(static_cast<std::size_t>(get_parameter("max_observed_points").as_int())),
  camera_model_(make_camera_model(*this))
{
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
    try {
        const auto mask_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat binary_mask;
        cv::threshold(mask_image->image, binary_mask, mask_threshold_, 255.0, cv::THRESH_BINARY);

        // 細線化
        cv::Mat skeleton_mask;
        cv::ximgproc::thinning(binary_mask, skeleton_mask, cv::ximgproc::THINNING_ZHANGSUEN);

        if (ground_projection_lut_.empty()) {
            ground_projection_lut_ = build_ground_projection_lut(camera_model_, pixel_step_, skeleton_mask.cols, skeleton_mask.rows);
        }

        const auto base_points = lane_pixels_to_base_points(
            skeleton_mask, ground_projection_lut_, mask_threshold_, max_observed_points_);

        lane_line_points_publisher_->publish(make_lane_line_point_cloud(base_points, msg->header.stamp));
        lane_line_marker_publisher_->publish(make_lane_line_marker_array(base_points, msg->header.stamp));
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "lane_line_publisher skipped: %s",
            error.what());
    }
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
