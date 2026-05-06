#pragma once

#include <unordered_map>
#include <vector>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <mapless_planning_msgs/msg/driving_corridor.hpp>
#include <mapless_planning_msgs/msg/road_segments.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lane_planner
{

struct LaneletConnection
{
    int original_lanelet_id{-1};
    std::vector<int> predecessor_lanelet_ids;
    std::vector<int> successor_lanelet_ids;
    std::vector<int> neighbor_lanelet_ids;
    bool goal_information{true};
};

// RoadSegments の各 Segment を lanelet::Lanelet に変換し、接続情報を構築する。
// out_lanelets[i] と out_connections[i] はインデックスベースで対応する。
void convert_segments_to_lanelets(
    const mapless_planning_msgs::msg::RoadSegments & msg,
    std::vector<lanelet::Lanelet> & out_lanelets,
    std::vector<LaneletConnection> & out_connections);

// ego 位置 (0,0) が含まれる lanelet のインデックスを返す。見つからない場合は -1。
int find_ego_lanelet_id(const std::vector<lanelet::Lanelet> & lanelets);

// lane_ids で指定した lanelet 群から DrivingCorridor を構築する。
mapless_planning_msgs::msg::DrivingCorridor create_driving_corridor(
    const std::vector<int> & lane_ids,
    const std::vector<lanelet::Lanelet> & lanelets);

// 全 lanelet の centerline・左右境界を LINE_STRIP マーカーとして生成する。
visualization_msgs::msg::MarkerArray create_road_model_markers(
    const std::vector<lanelet::Lanelet> & lanelets,
    const std::string & frame_id,
    const rclcpp::Time & stamp);

}  // namespace lane_planner
