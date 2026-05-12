#include "lane_planner/vectormap_lane_planner_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lane_planner
{
namespace
{

constexpr double EPSILON = 1.0e-6;

double distance_2d(
    const VectormapLanePlannerPlugin::Point2D & a,
    const VectormapLanePlannerPlugin::Point2D & b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double point_segment_distance_sq(
    const VectormapLanePlannerPlugin::Point2D & point,
    const VectormapLanePlannerPlugin::Point2D & start,
    const VectormapLanePlannerPlugin::Point2D & end)
{
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= EPSILON) {
        const double dx = point.x - start.x;
        const double dy = point.y - start.y;
        return dx * dx + dy * dy;
    }
    const double wx = point.x - start.x;
    const double wy = point.y - start.y;
    const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
    const double px = start.x + t * vx;
    const double py = start.y + t * vy;
    const double dx = point.x - px;
    const double dy = point.y - py;
    return dx * dx + dy * dy;
}

VectormapLanePlannerPlugin::PathPoint interpolate_raw_path(
    const std::vector<VectormapLanePlannerPlugin::Point2D> & points,
    const std::vector<double> & s_values,
    const double target_s,
    const uint64_t lanelet_id)
{
    if (points.size() != s_values.size() || points.size() < 2U) {
        throw std::runtime_error("raw path is invalid");
    }
    const double clamped_s = std::clamp(target_s, s_values.front(), s_values.back());
    const auto upper = std::upper_bound(s_values.begin(), s_values.end(), clamped_s);
    std::size_t index = 0U;
    if (upper == s_values.end()) {
        index = s_values.size() - 2U;
    } else if (upper != s_values.begin()) {
        index = static_cast<std::size_t>(std::distance(s_values.begin(), upper) - 1);
    }
    const double segment_length = s_values[index + 1U] - s_values[index];
    const double ratio = segment_length > EPSILON
        ? (clamped_s - s_values[index]) / segment_length : 0.0;
    const auto & p0 = points[index];
    const auto & p1 = points[index + 1U];
    const double x = p0.x + ratio * (p1.x - p0.x);
    const double y = p0.y + ratio * (p1.y - p0.y);
    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    return {clamped_s, x, y, yaw, lanelet_id};
}

}  // namespace

// ---------- LanePlannerPlugin interface ----------

void VectormapLanePlannerPlugin::initialize(rclcpp::Node & node)
{
    clock_ = node.get_clock();
    logger_ = node.get_logger();

    map_frame_id_ = node.get_parameter("map_frame_id").as_string();
    vector_map_topic_ = node.get_parameter("vector_map_topic").as_string();
    default_nav_cmd_ = node.get_parameter("default_nav_cmd").as_string();
    route_lanelet_ids_param_ = node.get_parameter("route_lanelet_ids").as_integer_array();
    nav_cmd_fallback_order_param_ = node.get_parameter("nav_cmd_fallback_order").as_string_array();
    global_path_resample_interval_m_ =
        node.get_parameter("global_path_resample_interval_m").as_double();
    max_centerline_connection_gap_m_ =
        node.get_parameter("max_centerline_connection_gap_m").as_double();
    off_route_distance_threshold_m_ =
        node.get_parameter("off_route_distance_threshold_m").as_double();
    route_lookahead_lanelet_count_ =
        node.get_parameter("route_lookahead_lanelet_count").as_int();

    if (route_lanelet_ids_param_.empty()) {
        throw std::invalid_argument("route_lanelet_ids must not be empty");
    }
    for (const auto id : route_lanelet_ids_param_) {
        if (id <= 0) {
            throw std::invalid_argument("route_lanelet_ids must contain positive lanelet ids");
        }
    }
    if (nav_cmd_fallback_order_param_.empty()) {
        throw std::invalid_argument("nav_cmd_fallback_order must not be empty");
    }
    if (global_path_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("global_path_resample_interval_m must be greater than 0");
    }
    if (off_route_distance_threshold_m_ <= 0.0) {
        throw std::invalid_argument("off_route_distance_threshold_m must be greater than 0");
    }
    if (route_lookahead_lanelet_count_ < 3) {
        throw std::invalid_argument("route_lookahead_lanelet_count must be at least 3");
    }

    last_nav_cmd_turn_ = parse_nav_cmd(default_nav_cmd_);
    nav_cmd_fallback_order_.reserve(nav_cmd_fallback_order_param_.size());
    for (const auto & cmd : nav_cmd_fallback_order_param_) {
        nav_cmd_fallback_order_.push_back(parse_nav_cmd(cmd));
    }

    vector_map_sub_ = node.create_subscription<vectormap_msgs::msg::VectorMap>(
        vector_map_topic_,
        rclcpp::QoS(10),
        [this](const vectormap_msgs::msg::VectorMap::SharedPtr msg) {
            vector_map_callback(msg);
        });

    RCLCPP_INFO(logger_, "VectormapLanePlannerPlugin initialized");
}

void VectormapLanePlannerPlugin::set_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(pose);
}

void VectormapLanePlannerPlugin::set_nav_command(const std::string & command)
{
    uint8_t turn = 0;
    try {
        turn = parse_nav_cmd(command);
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(logger_, "%s", e.what());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_nav_cmd_turn_ = turn;
    if (global_path_ready_ && latest_pose_) {
        const Point2D ego{
            latest_pose_->pose.pose.position.x,
            latest_pose_->pose.pose.position.y};
        rebuild_route_from_pose(ego, "nav_cmd");
    }
}

std::optional<nav_msgs::msg::Path> VectormapLanePlannerPlugin::compute_path(
    const rclcpp::Time & stamp)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_path_ready_ || !latest_pose_) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000,
            "waiting for vector map and localization pose");
        return std::nullopt;
    }

    const Point2D ego{
        latest_pose_->pose.pose.position.x,
        latest_pose_->pose.pose.position.y};

    if (pending_route_rebuild_) {
        rebuild_route_from_pose(ego, pending_route_rebuild_reason_);
        pending_route_rebuild_ = false;
        pending_route_rebuild_reason_.clear();
    } else {
        const auto [current_lanelet_id, distance] = find_nearest_lanelet_within_route(ego);
        if (current_lanelet_id != 0U) {
            if (distance > off_route_distance_threshold_m_) {
                rebuild_route_from_pose(ego, "out_of_route");
            } else {
                const auto route_it = std::find(
                    current_route_lanelet_ids_.begin(),
                    current_route_lanelet_ids_.end(),
                    current_lanelet_id);
                const std::size_t remaining = static_cast<std::size_t>(
                    std::distance(route_it, current_route_lanelet_ids_.end()));
                if (remaining == 1U) {
                    rebuild_route_from_lanelet(current_lanelet_id, "lookahead_extension");
                }
            }
        }
    }

    return make_path_message(stamp);
}

// ---------- Internal methods ----------

void VectormapLanePlannerPlugin::vector_map_callback(
    const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("VectorMap message must not be null");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (global_path_ready_) {
        return;
    }
    build_global_path_once(*msg);
}

void VectormapLanePlannerPlugin::build_global_path_once(
    const vectormap_msgs::msg::VectorMap & map_msg)
{
    if (map_msg.header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "VectorMap frame_id must be " + map_frame_id_ +
            ", got " + map_msg.header.frame_id);
    }

    lanelet_by_id_.clear();
    lanelet_by_id_.reserve(map_msg.lanelets.size());
    for (const auto & lanelet : map_msg.lanelets) {
        lanelet_by_id_.emplace(lanelet.id, lanelet);
    }

    std::unordered_map<uint64_t, vectormap_msgs::msg::LineString> line_string_by_id;
    line_string_by_id.reserve(map_msg.line_strings.size());
    for (const auto & ls : map_msg.line_strings) {
        line_string_by_id.emplace(ls.id, ls);
    }

    connection_edges_by_from_lanelet_id_.clear();
    connection_edges_by_from_lanelet_id_.reserve(map_msg.lane_connections.size());
    for (const auto & conn : map_msg.lane_connections) {
        connection_edges_by_from_lanelet_id_[conn.from_lanelet_id].push_back(
            RouteEdge{conn.to_lanelet_id, conn.turn_direction, conn.cost});
    }

    lanelet_centerline_points_by_id_.clear();
    lanelet_centerline_points_by_id_.reserve(lanelet_by_id_.size());
    for (const auto & [lanelet_id, lanelet] : lanelet_by_id_) {
        const auto ls_it = line_string_by_id.find(lanelet.centerline_id);
        if (ls_it == line_string_by_id.end() || ls_it->second.points.size() < 2U) {
            continue;
        }
        if (ls_it->second.line_type != vectormap_msgs::msg::LineString::TYPE_VIRTUAL_LINE ||
            ls_it->second.marking_type != vectormap_msgs::msg::LineString::MARKING_VIRTUAL)
        {
            throw std::runtime_error("route centerline must be virtual line");
        }
        std::vector<Point2D> pts;
        pts.reserve(ls_it->second.points.size());
        for (const auto & p : ls_it->second.points) {
            pts.push_back({p.x, p.y});
        }
        lanelet_centerline_points_by_id_.emplace(lanelet_id, std::move(pts));
    }

    std::size_t fallback_count = 0U;
    const auto route_ids = build_route_sequence_from_graph(
        static_cast<uint64_t>(route_lanelet_ids_param_.front()), fallback_count);
    if (fallback_count > 0U) {
        RCLCPP_WARN(
            logger_,
            "initial route used nav_cmd fallback %zu times: requested=%s",
            fallback_count,
            turn_direction_to_string(last_nav_cmd_turn_).c_str());
    }
    build_route_from_lanelet_ids(route_ids);
    path_frame_id_ = map_msg.header.frame_id;
    global_path_ready_ = true;
    RCLCPP_INFO(
        logger_,
        "built vector map global path: %zu points, %zu lanelets, loop=%s",
        global_samples_.size(),
        route_ids.size(),
        route_is_loop_ ? "true" : "false");
}

void VectormapLanePlannerPlugin::build_route_from_lanelet_ids(
    const std::vector<uint64_t> & route_lanelet_ids)
{
    if (route_lanelet_ids.empty()) {
        throw std::runtime_error("route lanelet sequence must not be empty");
    }

    std::vector<Point2D> route_points;
    std::vector<double> raw_s;
    lanelet_ranges_.clear();
    route_is_loop_ = false;
    route_points.reserve(route_lanelet_ids.size() * 16U);

    double accumulated_s = 0.0;
    for (std::size_t i = 0U; i < route_lanelet_ids.size(); ++i) {
        const uint64_t lanelet_id = route_lanelet_ids[i];
        if (lanelet_by_id_.find(lanelet_id) == lanelet_by_id_.end()) {
            throw std::runtime_error(
                "route lanelet id not found: " + std::to_string(lanelet_id));
        }

        if (i + 1U < route_lanelet_ids.size()) {
            const uint64_t next_id = route_lanelet_ids[i + 1U];
            const auto conn_it = connection_edges_by_from_lanelet_id_.find(lanelet_id);
            const bool connected = conn_it != connection_edges_by_from_lanelet_id_.end() &&
                std::any_of(
                    conn_it->second.begin(), conn_it->second.end(),
                    [next_id](const RouteEdge & e) { return e.to_lanelet_id == next_id; });
            if (!connected) {
                throw std::runtime_error(
                    "missing LaneConnection from " + std::to_string(lanelet_id) +
                    " to " + std::to_string(next_id));
            }
        }

        const auto cl_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (cl_it == lanelet_centerline_points_by_id_.end()) {
            throw std::runtime_error("route centerline points are not available");
        }
        const auto & pts = cl_it->second;
        if (!route_points.empty()) {
            const double gap = distance_2d(route_points.back(), pts.front());
            if (gap > max_centerline_connection_gap_m_) {
                throw std::runtime_error(
                    "centerline connection gap too large: " + std::to_string(gap));
            }
        }

        const double start_s = accumulated_s;
        for (const auto & p : pts) {
            if (!route_points.empty()) {
                const double ds = distance_2d(route_points.back(), p);
                if (ds <= EPSILON) {
                    continue;
                }
                accumulated_s += ds;
            }
            route_points.push_back(p);
            raw_s.push_back(accumulated_s);
        }
        lanelet_ranges_.push_back({lanelet_id, start_s, accumulated_s});
    }

    if (route_points.size() < 2U) {
        throw std::runtime_error("route must contain at least two centerline points");
    }

    const uint64_t first_id = route_lanelet_ids.front();
    const uint64_t last_id = route_lanelet_ids.back();
    const auto last_conn_it = connection_edges_by_from_lanelet_id_.find(last_id);
    route_is_loop_ = last_conn_it != connection_edges_by_from_lanelet_id_.end() &&
        std::any_of(
            last_conn_it->second.begin(), last_conn_it->second.end(),
            [first_id](const RouteEdge & e) { return e.to_lanelet_id == first_id; });

    if (route_is_loop_) {
        const double closing_gap = distance_2d(route_points.back(), route_points.front());
        if (closing_gap > max_centerline_connection_gap_m_) {
            throw std::runtime_error(
                "loop closing gap too large: " + std::to_string(closing_gap));
        }
        if (closing_gap > EPSILON) {
            accumulated_s += closing_gap;
            route_points.push_back(route_points.front());
            raw_s.push_back(accumulated_s);
            lanelet_ranges_.back().end_s = accumulated_s;
        }
    }

    global_samples_.clear();
    global_samples_.reserve(
        static_cast<std::size_t>(
            std::ceil(raw_s.back() / global_path_resample_interval_m_)) + 2U);
    for (double s = 0.0; s < raw_s.back(); s += global_path_resample_interval_m_) {
        global_samples_.push_back(
            interpolate_raw_path(route_points, raw_s, s, lanelet_at_s(s)));
    }
    global_samples_.push_back(
        interpolate_raw_path(route_points, raw_s, raw_s.back(), lanelet_at_s(raw_s.back())));
    current_route_lanelet_ids_ = route_lanelet_ids;
}

void VectormapLanePlannerPlugin::rebuild_route_from_lanelet(
    const uint64_t start_lanelet_id, const std::string & reason)
{
    std::size_t fallback_count = 0U;
    const auto route_ids = build_route_sequence_from_graph(start_lanelet_id, fallback_count);
    build_route_from_lanelet_ids(route_ids);
    RCLCPP_INFO(
        logger_,
        "rebuilt global path: reason=%s start=%lu lanelets=%zu points=%zu",
        reason.c_str(), start_lanelet_id, route_ids.size(), global_samples_.size());
    if (fallback_count > 0U) {
        RCLCPP_WARN(
            logger_, "route rebuild used fallback %zu times", fallback_count);
    }
}

bool VectormapLanePlannerPlugin::rebuild_route_from_pose(
    const Point2D & ego, const std::string & reason)
{
    const uint64_t start_id = find_nearest_lanelet_from_pose(ego);
    if (start_id == 0U) {
        throw std::runtime_error("failed to find nearest lanelet for route rebuild");
    }
    rebuild_route_from_lanelet(start_id, reason);
    return true;
}

std::vector<uint64_t> VectormapLanePlannerPlugin::build_route_sequence_from_graph(
    const uint64_t start_lanelet_id, std::size_t & fallback_count) const
{
    if (lanelet_centerline_points_by_id_.find(start_lanelet_id) ==
        lanelet_centerline_points_by_id_.end())
    {
        throw std::runtime_error(
            "route start lanelet centerline not available: " +
            std::to_string(start_lanelet_id));
    }

    fallback_count = 0U;
    std::vector<uint64_t> ids;
    ids.reserve(static_cast<std::size_t>(route_lookahead_lanelet_count_));
    ids.push_back(start_lanelet_id);

    while (ids.size() < static_cast<std::size_t>(route_lookahead_lanelet_count_)) {
        uint8_t selected_turn = 0;
        bool used_fallback = false;
        const uint64_t next_id = select_next_lanelet(
            ids.back(), last_nav_cmd_turn_, selected_turn, used_fallback);
        if (next_id == 0U || next_id == start_lanelet_id) {
            break;
        }
        if (std::find(ids.begin(), ids.end(), next_id) != ids.end()) {
            RCLCPP_WARN(
                logger_,
                "route cycle detected at lanelet %lu; stopping lookahead", next_id);
            break;
        }
        if (lanelet_centerline_points_by_id_.find(next_id) ==
            lanelet_centerline_points_by_id_.end())
        {
            throw std::runtime_error(
                "selected lanelet centerline not available: " + std::to_string(next_id));
        }
        ids.push_back(next_id);
        if (used_fallback) {
            ++fallback_count;
        }
    }
    return ids;
}

uint64_t VectormapLanePlannerPlugin::select_next_lanelet(
    const uint64_t from_lanelet_id,
    const uint8_t requested_turn,
    uint8_t & selected_turn,
    bool & used_fallback) const
{
    const auto edges_it = connection_edges_by_from_lanelet_id_.find(from_lanelet_id);
    if (edges_it == connection_edges_by_from_lanelet_id_.end() ||
        edges_it->second.empty())
    {
        return 0U;
    }

    const auto find_min_cost = [&edges = edges_it->second](uint8_t turn) {
        auto best = edges.end();
        for (auto it = edges.begin(); it != edges.end(); ++it) {
            if (it->turn_direction != turn) {
                continue;
            }
            if (best == edges.end() || it->cost < best->cost) {
                best = it;
            }
        }
        return best;
    };

    const auto req_it = find_min_cost(requested_turn);
    if (req_it != edges_it->second.end() &&
        req_it->turn_direction == requested_turn)
    {
        selected_turn = requested_turn;
        used_fallback = false;
        return req_it->to_lanelet_id;
    }

    for (const auto fallback_turn : nav_cmd_fallback_order_) {
        if (fallback_turn == requested_turn) {
            continue;
        }
        const auto fb_it = find_min_cost(fallback_turn);
        if (fb_it != edges_it->second.end() &&
            fb_it->turn_direction == fallback_turn)
        {
            selected_turn = fallback_turn;
            used_fallback = true;
            return fb_it->to_lanelet_id;
        }
    }
    return 0U;
}

uint64_t VectormapLanePlannerPlugin::find_nearest_lanelet_from_pose(
    const Point2D & point) const
{
    uint64_t best_id = 0U;
    double best_dist_sq = std::numeric_limits<double>::max();
    for (const auto & [lanelet_id, pts] : lanelet_centerline_points_by_id_) {
        if (pts.size() < 2U) {
            continue;
        }
        for (std::size_t i = 1U; i < pts.size(); ++i) {
            const double d = point_segment_distance_sq(point, pts[i - 1U], pts[i]);
            if (d < best_dist_sq) {
                best_dist_sq = d;
                best_id = lanelet_id;
            }
        }
    }
    return best_id;
}

std::pair<uint64_t, double> VectormapLanePlannerPlugin::find_nearest_lanelet_within_route(
    const Point2D & point) const
{
    uint64_t best_id = 0U;
    double best_dist_sq = std::numeric_limits<double>::max();
    for (const uint64_t lanelet_id : current_route_lanelet_ids_) {
        const auto cl_it = lanelet_centerline_points_by_id_.find(lanelet_id);
        if (cl_it == lanelet_centerline_points_by_id_.end() ||
            cl_it->second.size() < 2U)
        {
            continue;
        }
        const auto & pts = cl_it->second;
        for (std::size_t i = 1U; i < pts.size(); ++i) {
            const double d = point_segment_distance_sq(point, pts[i - 1U], pts[i]);
            if (d < best_dist_sq) {
                best_dist_sq = d;
                best_id = lanelet_id;
            }
        }
    }
    const double dist = best_id != 0U
        ? std::sqrt(best_dist_sq)
        : std::numeric_limits<double>::max();
    return {best_id, dist};
}

uint64_t VectormapLanePlannerPlugin::lanelet_at_s(const double s) const
{
    if (lanelet_ranges_.empty()) {
        return 0U;
    }
    const double ns = normalize_path_s(s);
    const auto it = std::find_if(
        lanelet_ranges_.begin(), lanelet_ranges_.end(),
        [ns](const LaneletRange & r) {
            return ns >= r.start_s && ns <= r.end_s;
        });
    return it != lanelet_ranges_.end()
        ? it->lanelet_id
        : lanelet_ranges_.back().lanelet_id;
}

double VectormapLanePlannerPlugin::normalize_path_s(const double s) const
{
    double path_length = 0.0;
    if (!global_samples_.empty()) {
        path_length = global_samples_.back().s;
    } else if (!lanelet_ranges_.empty()) {
        path_length = lanelet_ranges_.back().end_s;
    }
    if (path_length <= EPSILON) {
        return s;
    }
    if (!route_is_loop_) {
        return std::clamp(s, 0.0, path_length);
    }
    double n = std::fmod(s, path_length);
    if (n < 0.0) {
        n += path_length;
    }
    return n;
}

uint8_t VectormapLanePlannerPlugin::parse_nav_cmd(const std::string & command) const
{
    if (command == "straight") {
        return vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT;
    }
    if (command == "left") {
        return vectormap_msgs::msg::LaneConnection::TURN_LEFT;
    }
    if (command == "right") {
        return vectormap_msgs::msg::LaneConnection::TURN_RIGHT;
    }
    throw std::invalid_argument("nav_cmd must be straight, left, or right: " + command);
}

std::string VectormapLanePlannerPlugin::turn_direction_to_string(
    const uint8_t turn_direction) const
{
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_STRAIGHT) return "straight";
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_LEFT)     return "left";
    if (turn_direction == vectormap_msgs::msg::LaneConnection::TURN_RIGHT)    return "right";
    return "unknown";
}

nav_msgs::msg::Path VectormapLanePlannerPlugin::make_path_message(
    const rclcpp::Time & stamp) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = path_frame_id_.empty() ? map_frame_id_ : path_frame_id_;
    path.poses.reserve(global_samples_.size());
    for (const auto & pt : global_samples_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = yaw_to_quaternion(pt.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

geometry_msgs::msg::Quaternion VectormapLanePlannerPlugin::yaw_to_quaternion(const double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace lane_planner
