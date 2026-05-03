#pragma once

#include <string>

#include "vectormap_msgs/msg/vector_map.hpp"

namespace vectormap_server
{

vectormap_msgs::msg::VectorMap load_vector_map_from_osm(const std::string& map_path);

}  // namespace vectormap_server
