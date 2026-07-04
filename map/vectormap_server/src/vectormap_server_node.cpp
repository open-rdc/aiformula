#include "vectormap_server/vectormap_server_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "vectormap_server/osm_parser.hpp"
#include "vectormap_server/vectormap_visualizer.hpp"

namespace vectormap_server
{

VectormapServerNode::VectormapServerNode(const rclcpp::NodeOptions& options)
: VectormapServerNode("", options)
{
}

VectormapServerNode::VectormapServerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("vectormap_server_node", name_space, options),
  map_path_(get_parameter("map_path").as_string()),
  map_yaw_from_east_(get_parameter("map_yaw_from_east").as_double())
{
    if (!std::isfinite(map_yaw_from_east_)) {
        throw std::invalid_argument("map_yaw_from_east must be finite");
    }

    const std::string resolved_map_path = resolve_map_path(map_path_);
    map_msg_ = load_vector_map_from_osm(resolved_map_path);
    marker_array_ = create_vector_map_marker_array(map_msg_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    vector_map_publisher_ = this->create_publisher<vectormap_msgs::msg::VectorMap>(
        "vector_map", rclcpp::QoS(1).transient_local());
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "vector_map/visualize", rclcpp::QoS(1).transient_local());

    publish_static_transforms();
    publish_map();

    RCLCPP_INFO(
        this->get_logger(),
        "loaded vector map: path=%s, line_strings=%zu, lanelets=%zu, lane_connections=%zu, areas=%zu",
        resolved_map_path.c_str(),
        map_msg_.line_strings.size(),
        map_msg_.lanelets.size(),
        map_msg_.lane_connections.size(),
        map_msg_.areas.size());
}

std::string VectormapServerNode::resolve_map_path(const std::string& map_path)
{
    const std::filesystem::path path(map_path);
    const std::filesystem::path resolved_path = path.is_absolute()
        ? path
        : std::filesystem::path(ament_index_cpp::get_package_share_directory("vectormap_server")) /
            "config" / path;

    if (!std::filesystem::exists(resolved_path)) {
        throw std::runtime_error("vector map file does not exist: " + resolved_path.string());
    }
    if (!std::filesystem::is_regular_file(resolved_path)) {
        throw std::runtime_error("vector map path is not a file: " + resolved_path.string());
    }

    return resolved_path.string();
}

geometry_msgs::msg::TransformStamped VectormapServerNode::create_earth_to_map_transform() const
{
    const double half_yaw = map_yaw_from_east_ * 0.5;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "earth";
    transform.child_frame_id = map_msg_.header.frame_id;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(half_yaw);
    transform.transform.rotation.w = std::cos(half_yaw);
    return transform;
}

void VectormapServerNode::publish_static_transforms()
{
    const auto transform = create_earth_to_map_transform();
    static_tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(
        this->get_logger(),
        "published static tf %s -> %s: map_yaw_from_east=%.4f rad (%.2f deg)",
        transform.header.frame_id.c_str(),
        transform.child_frame_id.c_str(),
        map_yaw_from_east_,
        map_yaw_from_east_ * 180.0 / M_PI);
}

void VectormapServerNode::publish_map()
{
    const auto stamp = this->now();
    map_msg_.header.stamp = stamp;
    update_marker_array_stamp(marker_array_, stamp);

    vector_map_publisher_->publish(map_msg_);
    marker_array_publisher_->publish(marker_array_);
}

}
