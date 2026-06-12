#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vectormap_msgs/msg/vector_map.hpp"

namespace vectormap_server
{

visualization_msgs::msg::MarkerArray create_vector_map_marker_array(
    const vectormap_msgs::msg::VectorMap& map_msg);

void update_marker_array_stamp(
    visualization_msgs::msg::MarkerArray& marker_array,
    const builtin_interfaces::msg::Time& stamp);

}  // namespace vectormap_server
