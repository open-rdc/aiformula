#include "vectormap_server/vectormap_server_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <filesystem>
#include <functional>
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
  publish_period_ms_(get_parameter("publish_period_ms").as_int()),
  earth_frame_id_(get_parameter("earth_frame_id").as_string()),
  map_axis_convention_(get_parameter("map_axis_convention").as_string()),
  map_origin_pixel_x_(get_parameter("map_origin_pixel.x").as_double()),
  map_origin_pixel_y_(get_parameter("map_origin_pixel.y").as_double()),
  meter_per_pixel_(get_parameter("meter_per_pixel").as_double()),
  map_origin_lat_(get_parameter("map_origin_geodetic.latitude").as_double()),
  map_origin_lon_(get_parameter("map_origin_geodetic.longitude").as_double()),
  map_yaw_from_east_(get_parameter("map_yaw_from_east").as_double()),
  qos_(rclcpp::QoS(10))
{
    if (publish_period_ms_ <= 0) {
        throw std::invalid_argument("publish_period_ms must be greater than 0");
    }
    if (earth_frame_id_.empty()) {
        throw std::invalid_argument("earth_frame_id must not be empty");
    }
    if (meter_per_pixel_ <= 0.0) {
        throw std::invalid_argument("meter_per_pixel must be greater than 0");
    }
    if (map_axis_convention_ != "image_x_right_y_up") {
        throw std::invalid_argument("map_axis_convention must be image_x_right_y_up");
    }
    if (!std::isfinite(map_origin_lat_) || !std::isfinite(map_origin_lon_)) {
        throw std::invalid_argument("map_origin_geodetic latitude and longitude must be finite");
    }
    if (!std::isfinite(map_yaw_from_east_)) {
        throw std::invalid_argument("map_yaw_from_east must be finite");
    }

    const std::string resolved_map_path = resolve_map_path(map_path_);
    map_msg_ = load_vector_map_from_osm(resolved_map_path);
    marker_array_ = create_vector_map_marker_array(map_msg_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    vector_map_publisher_ = this->create_publisher<vectormap_msgs::msg::VectorMap>("vector_map", qos_);
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("vector_map/visualize", qos_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&VectormapServerNode::publish_callback, this));
    publish_static_transforms();

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
    // earth (ENU: x=East, y=North) -> map
    // map の +x 軸は East から map_yaw_from_east_ [rad] だけ CCW に回転した方向
    // earth フレームと map フレームは同一原点を持つため平行移動なし
    // TF の回転: map フレームが earth フレームに対して map_yaw_from_east_ だけ z 軸回りに回転している
    const double half_yaw = map_yaw_from_east_ * 0.5;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = earth_frame_id_;
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

void VectormapServerNode::publish_callback()
{
    const auto stamp = this->now();
    map_msg_.header.stamp = stamp;
    update_marker_array_stamp(marker_array_, stamp);

    vector_map_publisher_->publish(map_msg_);
    marker_array_publisher_->publish(marker_array_);
}

}  // namespace vectormap_server
