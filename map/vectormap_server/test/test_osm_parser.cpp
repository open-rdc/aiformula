#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "vectormap_server/osm_parser.hpp"

namespace
{

std::string write_temp_osm(const std::string& filename, const std::string& content)
{
    const std::string path = "/tmp/" + filename;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

}

TEST(OsmParser, RejectsMapWithNonMapFrameId)
{
    const std::string osm_xml =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<osm frame_id="world" map_id="test_map" map_version="1"/>
)";
    const std::string path = write_temp_osm("test_osm_parser_non_map_frame.osm", osm_xml);
    EXPECT_THROW(vectormap_server::load_vector_map_from_osm(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(OsmParser, AcceptsMapWithMapFrameIdAndNoElements)
{
    const std::string osm_xml =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<osm frame_id="map" map_id="test_map" map_version="1"/>
)";
    const std::string path = write_temp_osm("test_osm_parser_map_frame.osm", osm_xml);
    const auto map_msg = vectormap_server::load_vector_map_from_osm(path);
    EXPECT_EQ(map_msg.header.frame_id, "map");
    std::remove(path.c_str());
}

namespace
{

std::string make_lanelet_osm_xml(const std::string& centerline_way_tags)
{
    return
        R"(<?xml version="1.0" encoding="UTF-8"?>
<osm frame_id="map" map_id="test_map" map_version="1">
  <node id="1" x="0.0" y="0.0" z="0.0"/>
  <node id="2" x="1.0" y="0.0" z="0.0"/>
  <node id="3" x="0.0" y="1.0" z="0.0"/>
  <node id="4" x="1.0" y="1.0" z="0.0"/>
  <node id="5" x="0.0" y="0.5" z="0.0"/>
  <node id="6" x="1.0" y="0.5" z="0.0"/>
  <way id="10">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
    <tag k="marking_type" v="solid"/>
    <tag k="is_observable" v="true"/>
  </way>
  <way id="11">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
    <tag k="marking_type" v="solid"/>
    <tag k="is_observable" v="true"/>
  </way>
  <way id="12">
    <nd ref="5"/>
    <nd ref="6"/>
)" + centerline_way_tags + R"(
  </way>
  <relation id="100">
    <member type="way" role="left" ref="10"/>
    <member type="way" role="right" ref="11"/>
    <member type="way" role="centerline" ref="12"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="is_virtual" v="false"/>
  </relation>
</osm>
)";
}

}

TEST(OsmParser, RejectsLaneletWhoseCenterlineIsNotVirtualLine)
{
    const std::string centerline_tags =
        R"(    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
    <tag k="marking_type" v="solid"/>
    <tag k="is_observable" v="true"/>)";
    const std::string path = write_temp_osm(
        "test_osm_parser_invalid_centerline.osm", make_lanelet_osm_xml(centerline_tags));
    EXPECT_THROW(vectormap_server::load_vector_map_from_osm(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(OsmParser, AcceptsLaneletWhoseCenterlineIsVirtualLine)
{
    const std::string centerline_tags =
        R"(    <tag k="type" v="virtual_line"/>
    <tag k="subtype" v="virtual_line"/>
    <tag k="marking_type" v="virtual"/>
    <tag k="is_observable" v="false"/>)";
    const std::string path = write_temp_osm(
        "test_osm_parser_valid_centerline.osm", make_lanelet_osm_xml(centerline_tags));
    const auto map_msg = vectormap_server::load_vector_map_from_osm(path);
    ASSERT_EQ(map_msg.lanelets.size(), 1U);
    std::remove(path.c_str());
}
