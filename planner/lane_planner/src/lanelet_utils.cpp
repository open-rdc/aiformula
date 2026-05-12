#include "lane_planner/lanelet_utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lane_planner
{

void convert_segments_to_lanelets(
    const mapless_planning_msgs::msg::RoadSegments & msg,
    std::vector<lanelet::Lanelet> & out_lanelets,
    std::vector<LaneletConnection> & out_connections)
{
    out_lanelets.clear();
    out_connections.clear();
    out_lanelets.reserve(msg.segments.size());
    out_connections.reserve(msg.segments.size());

    // original segment id → index へのマッピング
    std::unordered_map<uint32_t, int> id_map;

    for (std::size_t idx = 0; idx < msg.segments.size(); ++idx) {
        const auto & seg = msg.segments[idx];

        // linestrings[0] = 左境界、linestrings[1] = 右境界
        std::array<lanelet::LineString3d, 2> ls;
        for (std::size_t li = 0; li < 2; ++li) {
            std::vector<lanelet::Point3d> pts;
            pts.reserve(seg.linestrings[li].poses.size());
            for (const auto & pose : seg.linestrings[li].poses) {
                pts.emplace_back(
                    lanelet::utils::getId(),
                    pose.position.x,
                    pose.position.y,
                    pose.position.z);
            }
            ls[li] = lanelet::LineString3d(lanelet::utils::getId(), std::move(pts));
        }

        out_lanelets.emplace_back(lanelet::utils::getId(), ls[0], ls[1]);

        LaneletConnection conn;
        conn.original_lanelet_id = static_cast<int>(seg.id);
        for (const auto id : seg.successor_segment_id) {
            conn.successor_lanelet_ids.push_back(static_cast<int>(id));
        }
        for (const auto id : seg.neighboring_segment_id) {
            conn.neighbor_lanelet_ids.push_back(static_cast<int>(id));
        }
        out_connections.push_back(conn);

        id_map[seg.id] = static_cast<int>(idx);
    }

    // original id をインデックスベース id に正規化
    auto remap = [&](std::vector<int> & ids) {
        for (auto & id : ids) {
            auto it = id_map.find(static_cast<uint32_t>(id));
            id = (it != id_map.end()) ? it->second : -1;
        }
    };
    for (auto & conn : out_connections) {
        remap(conn.successor_lanelet_ids);
        remap(conn.neighbor_lanelet_ids);
    }

    // predecessor を successor の逆引きで補完
    for (int i = 0; i < static_cast<int>(out_connections.size()); ++i) {
        for (int succ : out_connections[i].successor_lanelet_ids) {
            if (succ >= 0 && succ < static_cast<int>(out_connections.size())) {
                out_connections[succ].predecessor_lanelet_ids.push_back(i);
            }
        }
    }
}

int find_ego_lanelet_id(const std::vector<lanelet::Lanelet> & lanelets)
{
    const lanelet::BasicPoint2d ego{0.0, 0.0};
    for (int i = 0; i < static_cast<int>(lanelets.size()); ++i) {
        if (lanelet::geometry::inside(lanelets[i], ego)) {
            return i;
        }
    }
    return -1;
}

mapless_planning_msgs::msg::DrivingCorridor create_driving_corridor(
    const std::vector<int> & lane_ids,
    const std::vector<lanelet::Lanelet> & lanelets)
{
    mapless_planning_msgs::msg::DrivingCorridor corridor;

    for (const int id : lane_ids) {
        if (id < 0 || id >= static_cast<int>(lanelets.size())) {
            continue;
        }
        const auto & ll = lanelets[id];

        for (const auto & pt : ll.centerline()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            corridor.centerline.push_back(p);
        }
        for (const auto & pt : ll.leftBound()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            corridor.bound_left.push_back(p);
        }
        for (const auto & pt : ll.rightBound()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            corridor.bound_right.push_back(p);
        }
    }

    return corridor;
}

visualization_msgs::msg::MarkerArray create_road_model_markers(
    const std::vector<lanelet::Lanelet> & lanelets,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
{
    visualization_msgs::msg::MarkerArray arr;

    auto make_marker = [&](int id, const std::string & ns,
                           float r, float g, float b, float width)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = static_cast<double>(width);
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;
        return m;
    };

    for (int i = 0; i < static_cast<int>(lanelets.size()); ++i) {
        const auto & ll = lanelets[i];

        auto cl_marker = make_marker(i, "centerline", 0.0f, 1.0f, 0.0f, 0.05f);
        for (const auto & pt : ll.centerline()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            cl_marker.points.push_back(p);
        }
        if (!cl_marker.points.empty()) {
            arr.markers.push_back(cl_marker);
        }

        auto lb_marker = make_marker(i + 1000, "boundary", 1.0f, 1.0f, 1.0f, 0.03f);
        for (const auto & pt : ll.leftBound()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            lb_marker.points.push_back(p);
        }
        if (!lb_marker.points.empty()) {
            arr.markers.push_back(lb_marker);
        }

        auto rb_marker = make_marker(i + 2000, "boundary", 1.0f, 1.0f, 1.0f, 0.03f);
        for (const auto & pt : ll.rightBound()) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            rb_marker.points.push_back(p);
        }
        if (!rb_marker.points.empty()) {
            arr.markers.push_back(rb_marker);
        }
    }

    return arr;
}

}  // namespace lane_planner
