#include "traffic_signal_stop/traffic_signal_stop_node.hpp"

#include <cmath>
#include <functional>
#include <limits>

namespace traffic_signal_stop
{

using object_detection_msgs::msg::ObjectInfo;

TrafficSignalStopNode::TrafficSignalStopNode(const rclcpp::NodeOptions& options)
: TrafficSignalStopNode("", options)
{
}

TrafficSignalStopNode::TrafficSignalStopNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("traffic_signal_stop_node", name_space, options),
  stop_line_offset_m_(get_parameter("stop_line_offset_m").as_double())
{
    objects_subscription_ = create_subscription<object_detection_msgs::msg::ObjectInfoArray>(
        "/perception/objects", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&TrafficSignalStopNode::objects_callback, this, std::placeholders::_1));

    stop_point_publisher_ = create_publisher<speed_path_msgs::msg::StopPoint>("/planning/stop_point", rclcpp::QoS(1));
}

void TrafficSignalStopNode::objects_callback(const object_detection_msgs::msg::ObjectInfoArray::ConstSharedPtr msg)
{
    const ObjectInfo* panel = nullptr;
    for (const auto& object : msg->objects) {
        if (object.id == ObjectInfo::ID_PANEL_RED) {
            panel = &object;
            break;
        }
    }
    if (panel == nullptr) {
        return;
    }

    const double panel_bearing = std::atan2(panel->y, panel->x);

    const ObjectInfo* pylon = nullptr;
    double min_bearing_diff = std::numeric_limits<double>::infinity();
    for (const auto& object : msg->objects) {
        if (object.id != ObjectInfo::ID_PYLON) {
            continue;
        }
        const double bearing_diff = std::abs(std::atan2(object.y, object.x) - panel_bearing);
        if (bearing_diff < min_bearing_diff) {
            min_bearing_diff = bearing_diff;
            pylon = &object;
        }
    }
    if (pylon == nullptr) {
        return;
    }

    speed_path_msgs::msg::StopPoint stop_point;
    stop_point.distance = pylon->x + stop_line_offset_m_;
    stop_point_publisher_->publish(stop_point);
}

}
