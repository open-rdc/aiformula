#include "vectormap_server/vectormap_server_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "utilities/utils.hpp"
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
  map_path_(get_parameter("map_path").as_string())
{

    const std::string map_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("vectormap_server")) / "config" / map_path_;
    map_msg_ = load_vector_map_from_osm(map_path);
    marker_array_ = create_vector_map_marker_array(map_msg_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    vectormap_publisher_ = this->create_publisher<vectormap_msgs::msg::VectorMap>(
        "vector_map", rclcpp::QoS(1).transient_local());
    vectormap_visualize_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "vector_map/visualize", rclcpp::QoS(1).transient_local());

    publish_static_transforms();
    publish_map();

    RCLCPP_INFO(
        this->get_logger(),
        "loaded vector map: path=%s, line_strings=%zu, lanelets=%zu, lane_connections=%zu, areas=%zu",
        map_path.c_str(),
        map_msg_.line_strings.size(),
        map_msg_.lanelets.size(),
        map_msg_.lane_connections.size(),
        map_msg_.areas.size());
}

geometry_msgs::msg::TransformStamped VectormapServerNode::create_earth_to_map_transform() const
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "earth";
    transform.child_frame_id = map_msg_.header.frame_id;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = utils::yaw_to_quaternion(0.02686650); // 地図の向きを調整するオフセット
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
        0.02686650,
        utils::rtod(0.02686650));
}

void VectormapServerNode::publish_map()
{
    const auto stamp = this->now();
    map_msg_.header.stamp = stamp;
    update_marker_array_stamp(marker_array_, stamp);

    vectormap_publisher_->publish(map_msg_);
    vectormap_visualize_marker_publisher->publish(marker_array_);
}

}
