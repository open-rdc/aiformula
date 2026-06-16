#include "vectormap_server/vectormap_visualizer.hpp"

#include <std_msgs/msg/color_rgba.hpp>

#include "vectormap_msgs/msg/line_string.hpp"

namespace vectormap_server
{
namespace
{

using LineString = vectormap_msgs::msg::LineString;

std_msgs::msg::ColorRGBA make_color(
    const float red,
    const float green,
    const float blue,
    const float alpha)
{
    std_msgs::msg::ColorRGBA color;
    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = alpha;
    return color;
}

std_msgs::msg::ColorRGBA line_string_color(const LineString& line_string)
{
    if (line_string.marking_type == LineString::MARKING_VIRTUAL) {
        return make_color(0.0F, 0.8F, 1.0F, 0.45F);
    }
    if (line_string.line_type == LineString::TYPE_STOP_LINE) {
        return make_color(1.0F, 0.0F, 0.0F, 1.0F);
    }
    if (line_string.line_subtype == LineString::SUBTYPE_DASHED) {
        return make_color(1.0F, 0.8F, 0.0F, 1.0F);
    }
    if (line_string.line_subtype == LineString::SUBTYPE_ROAD_BORDER) {
        return make_color(0.1F, 0.4F, 1.0F, 1.0F);
    }
    return make_color(1.0F, 1.0F, 1.0F, 1.0F);
}

}  // namespace

visualization_msgs::msg::MarkerArray create_vector_map_marker_array(
    const vectormap_msgs::msg::VectorMap& map_msg)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header = map_msg.header;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int32_t marker_id = 0;
    for (const auto& line_string : map_msg.line_strings) {
        visualization_msgs::msg::Marker marker;
        marker.header = map_msg.header;
        marker.ns = "vector_map_line_strings";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = line_string.marking_type == LineString::MARKING_VIRTUAL ? 0.04 : 0.08;
        marker.color = line_string_color(line_string);
        marker.points = line_string.points;
        marker_array.markers.push_back(std::move(marker));
    }

    return marker_array;
}

void update_marker_array_stamp(
    visualization_msgs::msg::MarkerArray& marker_array,
    const builtin_interfaces::msg::Time& stamp)
{
    for (auto& marker : marker_array.markers) {
        marker.header.stamp = stamp;
    }
}

}  // namespace vectormap_server
