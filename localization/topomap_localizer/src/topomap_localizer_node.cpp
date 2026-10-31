#include "topomap_localizer/topomap_localizer_node.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "utilities/utils.hpp"

namespace topomap_localizer
{

TopomapLocalizerNode::TopomapLocalizerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("topomap_localizer_node", options),
  init_gnss_flag_(get_parameter("init_gnss_flag").as_bool())
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    rclcpp::SubscriptionOptions no_intra_process_options;
    no_intra_process_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    topomap_subscription_ = create_subscription<topomap_msgs::msg::TopoMap>("/map/topomap", rclcpp::QoS(1).transient_local(), std::bind(&TopomapLocalizerNode::topomap_callback, this, std::placeholders::_1), no_intra_process_options);
    // 自律走行を始めるたびに、その時点の姿勢で初期化し直す
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>("/autonomous", rclcpp::QoS(10), [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        initialized_ = initialized_ && !msg->data;
        autonomous_  = autonomous_ || msg->data;
    });
    gnss_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>("/vectornav/gnss", rclcpp::SensorDataQoS().keep_last(1), [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { gnss_ = msg; });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", rclcpp::SensorDataQoS().keep_last(1), [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_ = msg;
        imu_time_ = now();
    });
    nav_cmd_publisher_ = create_publisher<std_msgs::msg::String>("/planning/nav_cmd", rclcpp::QoS(10));

    timer_ = create_wall_timer(std::chrono::milliseconds(get_parameter("interval_ms").as_int()), std::bind(&TopomapLocalizerNode::timer_callback, this));
}

void TopomapLocalizerNode::topomap_callback(const topomap_msgs::msg::TopoMap::ConstSharedPtr msg)
{
    topomap_ = msg;

    const auto route = get_parameter("route").as_integer_array();
    const double straight_angle = utils::dtor(get_parameter("straight_angle_deg").as_double());
    const auto position = [&](uint64_t id) {
        for (const auto& node : msg->nodes) {
            if (node.id == id) {
                return node.position;
            }
        }
        return geometry_msgs::msg::Point();
    };

    // route の各ノードで指令を決め、straight 以外のノードを分岐目標として route 順に保持する
    targets_.clear();
    route_points_.clear();
    const size_t n = route.size();
    for (size_t i = 0; i < n; ++i) {
        route_points_.push_back(position(route[i]));
        const uint64_t prev = route[(i + n - 1) % n];
        const uint64_t cur = route[i];
        const uint64_t next = route[(i + 1) % n];
        const auto p_prev = position(prev);
        const auto p_cur = position(cur);
        const auto p_next = position(next);

        // 1. 進入してきた辺の向き
        const double hin = std::atan2(p_cur.y - p_prev.y, p_cur.x - p_prev.x);

        // 2. cur につながる prev 以外のノードを候補にして、進入方向からの旋回角を求める
        std::vector<uint64_t> candidates;
        for (const auto& edge : msg->edges) {
            if (edge.from_id == cur && edge.to_id != prev) {
                candidates.push_back(edge.to_id);
            } else if (edge.to_id == cur && edge.from_id != prev) {
                candidates.push_back(edge.from_id);
            }
        }
        const auto turn = [&](uint64_t id) {
            const auto p = position(id);
            return std::remainder(std::atan2(p.y - p_cur.y, p.x - p_cur.x) - hin, 2.0 * M_PI);
        };

        // 3. 候補が 1 つ以下なら直進
        if (candidates.size() <= 1) {
            continue;
        }

        // 4. next 以外で旋回角が最小の候補を比較対象にする
        const double tc = turn(next);
        double tr = 0.0;
        double best = INFINITY;
        for (const auto id : candidates) {
            if (id != next && std::abs(turn(id)) < best) {
                best = std::abs(turn(id));
                tr = turn(id);
            }
        }

        // 5. next が比較対象より直進寄りなら直進、そうでなければ左右を決める
        if (std::abs(tc) < std::abs(tr) && (tc * tr > 0.0 || std::abs(tc) < straight_angle)) {
            continue;
        }
        targets_.push_back({cur, p_cur, tc > tr ? "left" : "right", std::atan2(p_next.y - p_cur.y, p_next.x - p_cur.x), i});
    }
}

void TopomapLocalizerNode::timer_callback()
{
    // 1. topomap と odom->base_link から odom 上の姿勢を得る
    if (!topomap_ || !autonomous_ || !imu_ || !tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
        return;
    }
    const auto odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    const std::array<double, 2> p_o = {odom_to_base.transform.translation.x, odom_to_base.transform.translation.y};
    const double yaw_o = utils::yaw_from_quaternion(odom_to_base.transform.rotation);

    // 2. map 上の向きは INS の絶対方位
    const bool imu_alive = (now() - imu_time_).seconds() < get_parameter("imu_timeout_s").as_double();
    const double yaw_m = imu_alive || !initialized_ ? M_PI / 2.0 + utils::yaw_from_quaternion(imu_->orientation) : yaw_m_ + yaw_o - yaw_o_;

    // 3. 自律走行を始めた時点の位置を initial_position か GNSS から決める
    if (!initialized_) {
        auto initial_position = get_parameter("initial_position").as_double_array();
        if (init_gnss_flag_) {
            if (!gnss_ || gnss_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
                return;
            }
            // ノードの緯度経度と map 座標を基準に GNSS を map へ変換する
            constexpr double a = 6378137.0;
            constexpr double e2 = 6.69437999014e-3;
            const auto& ref = topomap_->nodes.front();
            const double lat0 = utils::dtor(ref.latitude);
            const double w = std::sqrt(1.0 - e2 * std::sin(lat0) * std::sin(lat0));
            initial_position[0] = ref.position.x + a / w * std::cos(lat0) * utils::dtor(gnss_->longitude - ref.longitude);
            initial_position[1] = ref.position.y + a * (1.0 - e2) / (w * w * w) * utils::dtor(gnss_->latitude - ref.latitude);
        }
        dr_ = {0.0, 0.0};
        tx_ = initial_position[0];
        ty_ = initial_position[1];
        anchors_.assign({{dr_, {initial_position[0], initial_position[1]}}});

        // 最寄りの route の辺の先にある最初の分岐目標から数える
        size_t edge = 0;
        double best = INFINITY;
        for (size_t i = 0; i < route_points_.size(); ++i) {
            const auto& p = route_points_[i];
            const auto& q = route_points_[(i + 1) % route_points_.size()];
            const double r = std::clamp(((initial_position[0] - p.x) * (q.x - p.x) + (initial_position[1] - p.y) * (q.y - p.y)) / ((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y)), 0.0, 1.0);
            const double d = std::hypot(initial_position[0] - p.x - r * (q.x - p.x), initial_position[1] - p.y - r * (q.y - p.y));
            if (d < best) {
                best = d;
                edge = i;
            }
        }
        k_ = 0;
        while (k_ < targets_.size() && targets_[k_].index <= edge) {
            ++k_;
        }
        active_ = false;
        initialized_ = true;
    } else {
        // 4. odom 上の移動量を車体座標に直し、map 上の向きで積算する
        const double dx = p_o[0] - p_o_[0];
        const double dy = p_o[1] - p_o_[1];
        const double forward = std::cos(yaw_o_) * dx + std::sin(yaw_o_) * dy;
        const double left = -std::sin(yaw_o_) * dx + std::cos(yaw_o_) * dy;
        const double yaw_mid = yaw_m_ + 0.5 * std::remainder(yaw_m - yaw_m_, 2.0 * M_PI);
        dr_[0] += std::cos(yaw_mid) * forward - std::sin(yaw_mid) * left;
        dr_[1] += std::sin(yaw_mid) * forward + std::cos(yaw_mid) * left;
    }
    p_o_ = p_o;
    yaw_o_ = yaw_o;
    yaw_m_ = yaw_m;

    // 5. map 上の自車位置を求める
    const double x_m = dr_[0] + tx_;
    const double y_m = dr_[1] + ty_;

    // 6. 分岐目標への接近と通過を判定する
    const Target* g = targets_.empty() ? nullptr : &targets_[k_ % targets_.size()];
    if (g) {
        const double d = std::hypot(x_m - g->position.x, y_m - g->position.y);
        const double trigger = get_parameter("trigger_distance_m").as_double();
        const double sign = g->command == "left" ? 1.0 : -1.0;
        if (!active_ && d < trigger) {
            active_ = true;
            yaw_in_ = yaw_m;
        } else if (active_ && sign * std::remainder(yaw_m - yaw_in_, 2.0 * M_PI) >= utils::dtor(get_parameter("turn_angle_deg").as_double()) &&
                   std::abs(std::remainder(yaw_m - g->hout, 2.0 * M_PI)) < utils::dtor(get_parameter("heading_tolerance_deg").as_double())) {
            // 向きは絶対方位で決まっているので、直近の分岐通過との位置ずれの平均だけを補正する
            anchors_.push_back({dr_, {g->position.x, g->position.y}});
            const size_t window = std::min(anchors_.size(), static_cast<size_t>(get_parameter("anchor_window").as_int()));
            tx_ = 0.0;
            ty_ = 0.0;
            for (auto it = anchors_.end() - window; it != anchors_.end(); ++it) {
                tx_ += (it->second[0] - it->first[0]) / window;
                ty_ += (it->second[1] - it->first[1]) / window;
            }
            active_ = false;
            ++k_;
        } else if (active_ && d > trigger) {
            active_ = false;
            ++k_;
        }
    }

    // 7. 指令が変わったときだけ publish する
    std_msgs::msg::String command;
    command.data = active_ ? g->command : "straight";
    if (command.data != command_) {
        command_ = command.data;
        nav_cmd_publisher_->publish(command);
        RCLCPP_INFO(get_logger(), "nav_cmd: %s (node %lu)", command_.c_str(), g ? g->id : 0UL);
    }

    // 8. map 上の自車姿勢と odom->base_link が一致するように map->odom を broadcast する
    const double yaw_mo = yaw_m - yaw_o;
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = x_m - (std::cos(yaw_mo) * p_o[0] - std::sin(yaw_mo) * p_o[1]);
    map_to_odom.transform.translation.y = y_m - (std::sin(yaw_mo) * p_o[0] + std::cos(yaw_mo) * p_o[1]);
    map_to_odom.transform.rotation = utils::yaw_to_quaternion(yaw_mo);
    tf_broadcaster_->sendTransform(map_to_odom);
}

}
