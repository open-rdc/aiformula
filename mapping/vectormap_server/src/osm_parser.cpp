#include "vectormap_server/osm_parser.hpp"

#include <tinyxml2.h>

#include <charconv>
#include <cstdlib>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <geometry_msgs/msg/point.hpp>

#include "vectormap_msgs/msg/lane_connection.hpp"
#include "vectormap_msgs/msg/lanelet.hpp"
#include "vectormap_msgs/msg/line_string.hpp"
#include "vectormap_msgs/msg/map_area.hpp"

namespace vectormap_server
{
namespace
{

using LaneConnection = vectormap_msgs::msg::LaneConnection;
using Lanelet = vectormap_msgs::msg::Lanelet;
using LineString = vectormap_msgs::msg::LineString;
using MapArea = vectormap_msgs::msg::MapArea;

std::string make_error(const std::string& context, const std::string& detail)
{
    return context + ": " + detail;
}

const char* required_attribute(
    const tinyxml2::XMLElement& element,
    const char* name,
    const std::string& context)
{
    const char* value = element.Attribute(name);
    if (value == nullptr) {
        throw std::runtime_error(make_error(context, std::string("missing attribute ") + name));
    }
    return value;
}

uint64_t parse_uint64(const char* value, const std::string& context)
{
    uint64_t result = 0;
    const std::string text(value);
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(begin, end, result);
    if (ec != std::errc() || ptr != end) {
        throw std::runtime_error(make_error(context, "invalid uint64 value " + text));
    }
    return result;
}

double parse_double(const char* value, const std::string& context)
{
    char* end = nullptr;
    const double result = std::strtod(value, &end);
    if (end == value || *end != '\0') {
        throw std::runtime_error(make_error(context, std::string("invalid double value ") + value));
    }
    return result;
}

bool parse_bool(const std::string& value, const std::string& context)
{
    if (value == "true") {
        return true;
    }
    if (value == "false") {
        return false;
    }
    throw std::runtime_error(make_error(context, "invalid bool value " + value));
}

std::unordered_map<std::string, std::string> read_tags(
    const tinyxml2::XMLElement& element,
    const std::string& context)
{
    std::unordered_map<std::string, std::string> tags;
    for (const tinyxml2::XMLElement* tag = element.FirstChildElement("tag");
        tag != nullptr;
        tag = tag->NextSiblingElement("tag"))
    {
        const std::string key(required_attribute(*tag, "k", context + " tag"));
        const std::string value(required_attribute(*tag, "v", context + " tag " + key));
        if (!tags.emplace(key, value).second) {
            throw std::runtime_error(make_error(context, "duplicate tag " + key));
        }
    }
    return tags;
}

std::string required_tag(
    const std::unordered_map<std::string, std::string>& tags,
    const std::string& key,
    const std::string& context)
{
    const auto iter = tags.find(key);
    if (iter == tags.end()) {
        throw std::runtime_error(make_error(context, "missing tag " + key));
    }
    return iter->second;
}

uint8_t parse_line_type(const std::string& value, const std::string& context)
{
    if (value == "line_thin") {
        return LineString::TYPE_LINE_THIN;
    }
    if (value == "virtual_line") {
        return LineString::TYPE_VIRTUAL_LINE;
    }
    if (value == "stop_line") {
        return LineString::TYPE_STOP_LINE;
    }
    throw std::runtime_error(make_error(context, "unknown line type " + value));
}

uint8_t parse_line_subtype(const std::string& value, const std::string& context)
{
    if (value == "solid") {
        return LineString::SUBTYPE_SOLID;
    }
    if (value == "dashed") {
        return LineString::SUBTYPE_DASHED;
    }
    if (value == "road_border") {
        return LineString::SUBTYPE_ROAD_BORDER;
    }
    if (value == "stop_line") {
        return LineString::SUBTYPE_STOP_LINE;
    }
    if (value == "virtual_line") {
        return LineString::SUBTYPE_VIRTUAL_LINE;
    }
    throw std::runtime_error(make_error(context, "unknown line subtype " + value));
}

uint8_t parse_marking_type(const std::string& value, const std::string& context)
{
    if (value == "solid") {
        return LineString::MARKING_SOLID;
    }
    if (value == "dashed") {
        return LineString::MARKING_DASHED;
    }
    if (value == "virtual") {
        return LineString::MARKING_VIRTUAL;
    }
    throw std::runtime_error(make_error(context, "unknown marking type " + value));
}

uint8_t parse_lanelet_subtype(const std::string& value, const std::string& context)
{
    if (value == "road") {
        return Lanelet::SUBTYPE_ROAD;
    }
    if (value == "intersection") {
        return Lanelet::SUBTYPE_INTERSECTION;
    }
    throw std::runtime_error(make_error(context, "unknown lanelet subtype " + value));
}

uint8_t parse_area_subtype(const std::string& value, const std::string& context)
{
    if (value == "crosswalk") {
        return MapArea::SUBTYPE_CROSSWALK;
    }
    throw std::runtime_error(make_error(context, "unknown area subtype " + value));
}

uint8_t parse_turn_direction(const std::string& value, const std::string& context)
{
    if (value == "unknown") {
        return LaneConnection::TURN_UNKNOWN;
    }
    if (value == "straight") {
        return LaneConnection::TURN_STRAIGHT;
    }
    if (value == "left") {
        return LaneConnection::TURN_LEFT;
    }
    if (value == "right") {
        return LaneConnection::TURN_RIGHT;
    }
    if (value == "merge") {
        return LaneConnection::TURN_MERGE;
    }
    if (value == "branch") {
        return LaneConnection::TURN_BRANCH;
    }
    if (value == "u_turn") {
        return LaneConnection::TURN_U_TURN;
    }
    throw std::runtime_error(make_error(context, "unknown turn_direction " + value));
}

void validate_line_tags(const LineString& line_string, const std::string& context)
{
    if (line_string.line_type == LineString::TYPE_LINE_THIN) {
        const bool valid_subtype =
            line_string.line_subtype == LineString::SUBTYPE_SOLID ||
            line_string.line_subtype == LineString::SUBTYPE_DASHED ||
            line_string.line_subtype == LineString::SUBTYPE_ROAD_BORDER;
        const bool valid_marking =
            line_string.marking_type == LineString::MARKING_SOLID ||
            line_string.marking_type == LineString::MARKING_DASHED;
        if (!valid_subtype || !valid_marking) {
            throw std::runtime_error(make_error(context, "invalid line_thin tag combination"));
        }
        return;
    }

    if (line_string.line_type == LineString::TYPE_VIRTUAL_LINE) {
        if (line_string.line_subtype != LineString::SUBTYPE_VIRTUAL_LINE ||
            line_string.marking_type != LineString::MARKING_VIRTUAL ||
            line_string.is_observable)
        {
            throw std::runtime_error(make_error(context, "invalid virtual_line tag combination"));
        }
        return;
    }

    if (line_string.line_type == LineString::TYPE_STOP_LINE) {
        if (line_string.line_subtype != LineString::SUBTYPE_STOP_LINE ||
            line_string.marking_type == LineString::MARKING_VIRTUAL)
        {
            throw std::runtime_error(make_error(context, "invalid stop_line tag combination"));
        }
        return;
    }

    throw std::runtime_error(make_error(context, "unsupported line type"));
}

uint64_t required_member_ref(
    const tinyxml2::XMLElement& relation,
    const std::string& role,
    const std::string& member_type,
    const std::string& context)
{
    bool found = false;
    uint64_t result = 0;
    for (const tinyxml2::XMLElement* member = relation.FirstChildElement("member");
        member != nullptr;
        member = member->NextSiblingElement("member"))
    {
        const std::string current_role(required_attribute(*member, "role", context + " member"));
        if (current_role != role) {
            continue;
        }
        const std::string current_type(required_attribute(*member, "type", context + " member " + role));
        if (current_type != member_type) {
            throw std::runtime_error(make_error(
                context,
                "member " + role + " must be type " + member_type + ", got " + current_type));
        }
        if (found) {
            throw std::runtime_error(make_error(context, "duplicate member role " + role));
        }
        result = parse_uint64(required_attribute(*member, "ref", context + " member " + role), context);
        found = true;
    }
    if (!found) {
        throw std::runtime_error(make_error(context, "missing member role " + role));
    }
    return result;
}

void require_id_exists(
    const std::unordered_set<uint64_t>& ids,
    const uint64_t id,
    const std::string& context)
{
    if (ids.find(id) == ids.end()) {
        throw std::runtime_error(make_error(context, "missing referenced id " + std::to_string(id)));
    }
}

}  // namespace

vectormap_msgs::msg::VectorMap load_vector_map_from_osm(const std::string& map_path)
{
    tinyxml2::XMLDocument document;
    const tinyxml2::XMLError load_result = document.LoadFile(map_path.c_str());
    if (load_result != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error(
            "failed to load vector map " + map_path + ": " + document.ErrorStr());
    }

    const tinyxml2::XMLElement* root = document.RootElement();
    if (root == nullptr || std::string(root->Name()) != "osm") {
        throw std::runtime_error("vector map root element must be osm");
    }

    vectormap_msgs::msg::VectorMap map_msg;
    map_msg.header.frame_id = required_attribute(*root, "frame_id", "osm");
    map_msg.map_id = required_attribute(*root, "map_id", "osm");
    map_msg.map_version = required_attribute(*root, "map_version", "osm");

    std::unordered_map<uint64_t, geometry_msgs::msg::Point> nodes;
    for (const tinyxml2::XMLElement* node = root->FirstChildElement("node");
        node != nullptr;
        node = node->NextSiblingElement("node"))
    {
        const uint64_t id = parse_uint64(required_attribute(*node, "id", "node"), "node");
        geometry_msgs::msg::Point point;
        point.x = parse_double(required_attribute(*node, "x", "node " + std::to_string(id)), "node");
        point.y = parse_double(required_attribute(*node, "y", "node " + std::to_string(id)), "node");
        point.z = parse_double(required_attribute(*node, "z", "node " + std::to_string(id)), "node");
        if (!nodes.emplace(id, point).second) {
            throw std::runtime_error("duplicate node id " + std::to_string(id));
        }
    }

    std::unordered_set<uint64_t> line_string_ids;
    for (const tinyxml2::XMLElement* way = root->FirstChildElement("way");
        way != nullptr;
        way = way->NextSiblingElement("way"))
    {
        const uint64_t id = parse_uint64(required_attribute(*way, "id", "way"), "way");
        const std::string context = "way " + std::to_string(id);
        const auto tags = read_tags(*way, context);

        LineString line_string;
        line_string.id = id;
        line_string.line_type = parse_line_type(required_tag(tags, "type", context), context);
        line_string.line_subtype = parse_line_subtype(required_tag(tags, "subtype", context), context);
        line_string.marking_type = parse_marking_type(required_tag(tags, "marking_type", context), context);
        line_string.is_observable = parse_bool(required_tag(tags, "is_observable", context), context);
        validate_line_tags(line_string, context);

        for (const tinyxml2::XMLElement* nd = way->FirstChildElement("nd");
            nd != nullptr;
            nd = nd->NextSiblingElement("nd"))
        {
            const uint64_t ref = parse_uint64(required_attribute(*nd, "ref", context + " nd"), context);
            const auto point_iter = nodes.find(ref);
            if (point_iter == nodes.end()) {
                throw std::runtime_error(make_error(context, "missing node ref " + std::to_string(ref)));
            }
            line_string.points.push_back(point_iter->second);
        }
        if (line_string.points.size() < 2) {
            throw std::runtime_error(make_error(context, "LineString must contain at least two points"));
        }
        if (!line_string_ids.insert(id).second) {
            throw std::runtime_error("duplicate way id " + std::to_string(id));
        }
        map_msg.line_strings.push_back(std::move(line_string));
    }

    std::unordered_set<uint64_t> lanelet_ids;
    std::unordered_set<uint64_t> relation_ids;
    for (const tinyxml2::XMLElement* relation = root->FirstChildElement("relation");
        relation != nullptr;
        relation = relation->NextSiblingElement("relation"))
    {
        const uint64_t id = parse_uint64(required_attribute(*relation, "id", "relation"), "relation");
        const std::string context = "relation " + std::to_string(id);
        if (!relation_ids.insert(id).second) {
            throw std::runtime_error("duplicate relation id " + std::to_string(id));
        }
        const auto tags = read_tags(*relation, context);
        const std::string type = required_tag(tags, "type", context);

        if (type == "lanelet") {
            Lanelet lanelet;
            lanelet.id = id;
            lanelet.subtype = parse_lanelet_subtype(required_tag(tags, "subtype", context), context);
            lanelet.is_virtual = parse_bool(required_tag(tags, "is_virtual", context), context);
            lanelet.left_line_id = required_member_ref(*relation, "left", "way", context);
            lanelet.right_line_id = required_member_ref(*relation, "right", "way", context);
            lanelet.centerline_id = required_member_ref(*relation, "centerline", "way", context);
            require_id_exists(line_string_ids, lanelet.left_line_id, context);
            require_id_exists(line_string_ids, lanelet.right_line_id, context);
            require_id_exists(line_string_ids, lanelet.centerline_id, context);
            if (!lanelet_ids.insert(id).second) {
                throw std::runtime_error("duplicate lanelet id " + std::to_string(id));
            }
            map_msg.lanelets.push_back(lanelet);
        } else if (type == "area") {
            MapArea area;
            area.id = id;
            area.subtype = parse_area_subtype(required_tag(tags, "subtype", context), context);
            area.outer_line_id = required_member_ref(*relation, "outer", "way", context);
            require_id_exists(line_string_ids, area.outer_line_id, context);
            map_msg.areas.push_back(area);
        } else if (type == "lane_connection") {
            LaneConnection lane_connection;
            lane_connection.id = id;
            lane_connection.from_lanelet_id = required_member_ref(*relation, "from", "relation", context);
            lane_connection.to_lanelet_id = required_member_ref(*relation, "to", "relation", context);
            lane_connection.turn_direction =
                parse_turn_direction(required_tag(tags, "turn_direction", context), context);
            lane_connection.cost =
                parse_double(required_tag(tags, "cost", context).c_str(), context + " cost");
            map_msg.lane_connections.push_back(lane_connection);
        } else {
            throw std::runtime_error(make_error(context, "unknown relation type " + type));
        }
    }

    for (const auto& lane_connection : map_msg.lane_connections) {
        const std::string context = "lane_connection " + std::to_string(lane_connection.id);
        require_id_exists(lanelet_ids, lane_connection.from_lanelet_id, context);
        require_id_exists(lanelet_ids, lane_connection.to_lanelet_id, context);
    }

    return map_msg;
}

}  // namespace vectormap_server
