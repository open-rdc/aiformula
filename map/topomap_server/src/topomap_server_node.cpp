#include "topomap_server/topomap_server_node.hpp"

#include <tinyxml2.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>

namespace topomap_server
{

TopomapServerNode::TopomapServerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("topomap_server_node", options)
{
    const std::string map_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("topomap_server")) / "config" / get_parameter("map_path").as_string();

    tinyxml2::XMLDocument document;
    document.LoadFile(map_path.c_str());

    topomap_msgs::msg::TopoMap topomap;
    topomap.header.frame_id = "map";
    topomap.header.stamp = now();

    // 1. node を TopoNode に変換する
    for (auto* node = document.RootElement()->FirstChildElement("node"); node; node = node->NextSiblingElement("node")) {
        topomap_msgs::msg::TopoNode topo_node;
        topo_node.id = node->Unsigned64Attribute("id");
        topo_node.latitude = node->DoubleAttribute("lat");
        topo_node.longitude = node->DoubleAttribute("lon");
        for (auto* tag = node->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            const std::string key = tag->Attribute("k");
            if (key == "local_x") {
                topo_node.position.x = tag->DoubleAttribute("v");
            } else if (key == "local_y") {
                topo_node.position.y = tag->DoubleAttribute("v");
            }
        }
        topomap.nodes.push_back(topo_node);
    }

    // 2. type=topo_edge の way を連続する nd の組ごとの TopoEdge に変換する
    for (auto* way = document.RootElement()->FirstChildElement("way"); way; way = way->NextSiblingElement("way")) {
        bool is_edge = false;
        for (auto* tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            is_edge |= std::string(tag->Attribute("k")) == "type" && std::string(tag->Attribute("v")) == "topo_edge";
        }
        if (!is_edge) {
            continue;
        }
        for (auto* nd = way->FirstChildElement("nd"); nd && nd->NextSiblingElement("nd"); nd = nd->NextSiblingElement("nd")) {
            topomap_msgs::msg::TopoEdge edge;
            edge.from_id = nd->Unsigned64Attribute("ref");
            edge.to_id = nd->NextSiblingElement("nd")->Unsigned64Attribute("ref");
            topomap.edges.push_back(edge);
        }
    }

    // 3. transient_local で 1 回だけ publish する
    rclcpp::PublisherOptions no_intra_process_options;
    no_intra_process_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    topomap_publisher_ = create_publisher<topomap_msgs::msg::TopoMap>("/map/topomap", rclcpp::QoS(1).transient_local(), no_intra_process_options);
    topomap_publisher_->publish(topomap);

    RCLCPP_INFO(get_logger(), "loaded topomap: nodes=%zu, edges=%zu", topomap.nodes.size(), topomap.edges.size());
}

}
