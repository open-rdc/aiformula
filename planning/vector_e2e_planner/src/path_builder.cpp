#include "vector_e2e_planner/path_builder.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <utilities/utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vector_e2e_planner {

namespace {

using Path = std::array<Point2D, num_path_points>;

bool is_finite(const Path& path) {
    return std::all_of(path.begin(), path.end(), [](const Point2D& p) { return std::isfinite(p.x) && std::isfinite(p.y); });
}

Path to_base(const Path& points, const Pose2D& pose) {
    const double c = std::cos(-pose.yaw);
    const double s = std::sin(-pose.yaw);
    Path         out{};
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double rx = points[i].x - pose.x;
        const double ry = points[i].y - pose.y;
        out[i]          = Point2D{rx * c - ry * s, rx * s + ry * c};
    }
    return out;
}

Path to_world_array(const Path& points, const Pose2D& pose) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    Path         out{};
    for (std::size_t i = 0; i < points.size(); ++i) {
        out[i] = Point2D{pose.x + c * points[i].x - s * points[i].y, pose.y + s * points[i].x + c * points[i].y};
    }
    return out;
}

Point2D catmull_rom(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, const double t) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const auto   axis = [&](const double a, const double b, const double c, const double d) {
        return 0.5 * ((2.0 * b) + (-a + c) * t + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2 + (-a + 3.0 * b - 3.0 * c + d) * t3);
    };
    return Point2D{axis(p0.x, p1.x, p2.x, p3.x), axis(p0.y, p1.y, p2.y, p3.y)};
}

}  // namespace

std::optional<Slot> parse_slot(const std::string& name) {
    if (name == "straight") {
        return Slot::Straight;
    }
    if (name == "left") {
        return Slot::Left;
    }
    if (name == "right") {
        return Slot::Right;
    }
    return std::nullopt;
}

const char* slot_name(const Slot slot) {
    switch (slot) {
        case Slot::Straight:
            return "straight";
        case Slot::Left:
            return "left";
        case Slot::Right:
            break;
    }
    return "right";
}

Slot select_slot(const Slot commanded, const std::array<double, num_slots>& branch_exist, const double exist_threshold) {
    if (branch_exist[static_cast<std::size_t>(commanded)] < exist_threshold) {
        return Slot::Straight;
    }
    return commanded;
}

std::vector<Point2D> valid_prefix(
    const std::array<Point2D, num_path_points>& path,
    const std::array<double, num_path_points>&  valid_prob,
    const double                                threshold) {
    std::vector<Point2D> kept;
    kept.reserve(num_path_points);
    for (std::size_t i = 0; i < num_path_points; ++i) {
        if (!(valid_prob[i] >= threshold) || !std::isfinite(path[i].x) || !std::isfinite(path[i].y)) {
            break;
        }
        kept.push_back(path[i]);
    }
    return kept;
}

PathSmoother::PathSmoother(const double alpha) : alpha_(alpha) {
}

std::array<Point2D, num_path_points> PathSmoother::step(const Path& path, const Slot slot, const Pose2D& odom_T_base) {
    const bool valid = is_finite(path);

    std::optional<Path> previous_base;
    if (previous_world_ && previous_slot_ && *previous_slot_ == slot && valid) {
        previous_base = to_base(*previous_world_, odom_T_base);
    }
    Path out = path;
    if (previous_base && alpha_ < 1.0) {
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = Point2D{
                alpha_ * path[i].x + (1.0 - alpha_) * (*previous_base)[i].x,
                alpha_ * path[i].y + (1.0 - alpha_) * (*previous_base)[i].y};
        }
    }
    if (valid) {
        previous_world_ = to_world_array(out, odom_T_base);
    } else {
        previous_world_.reset();
    }
    previous_slot_ = slot;
    return out;
}

std::vector<Point2D> build_output_points(const std::vector<Point2D>& points, const double interval) {
    std::vector<Point2D> base;
    base.reserve(points.size() + 1U);
    if (points.empty()) {
        return base;
    }
    // 経路を車両位置から始める。local_planner の出力も自車付近から始まり、停止距離の基準になる
    if (std::hypot(points.front().x, points.front().y) > 0.05) {
        base.push_back(Point2D{0.0, 0.0});
    }
    for (const Point2D& point : points) {
        // 同一点が並ぶと弧長が 0 の区間ができ、速度計画が割り算で発散する
        if (base.empty() || std::hypot(point.x - base.back().x, point.y - base.back().y) > 1e-3) {
            base.push_back(point);
        }
    }
    if (interval <= 0.0 || base.size() < 2U) {
        return base;
    }

    std::vector<Point2D> dense;
    const std::size_t    last = base.size() - 1U;
    for (std::size_t i = 0; i < last; ++i) {
        const Point2D& p0 = base[i == 0 ? 0 : i - 1];
        const Point2D& p1 = base[i];
        const Point2D& p2 = base[i + 1];
        const Point2D& p3 = base[std::min(i + 2, last)];
        const double   length = std::hypot(p2.x - p1.x, p2.y - p1.y);
        const std::size_t sub = std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length / interval)));
        for (std::size_t k = 0; k < sub; ++k) {
            dense.push_back(catmull_rom(p0, p1, p2, p3, static_cast<double>(k) / static_cast<double>(sub)));
        }
    }
    dense.push_back(base.back());
    return dense;
}

std::optional<double> stop_distance(const std::vector<Point2D>& points, const Point2D& target, const double offset) {
    if (points.size() < 2U) {
        return std::nullopt;
    }
    double best_dist = std::numeric_limits<double>::infinity();
    double best_arc  = 0.0;
    double arc_start = 0.0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const double sx      = points[i + 1].x - points[i].x;
        const double sy      = points[i + 1].y - points[i].y;
        const double length  = std::sqrt(sx * sx + sy * sy);
        const double length2 = std::max(sx * sx + sy * sy, 1e-12);
        const double t_raw   = ((target.x - points[i].x) * sx + (target.y - points[i].y) * sy) / length2;
        double       t       = std::min(std::max(t_raw, 0.0), 1.0);
        const double dx      = target.x - (points[i].x + t * sx);
        const double dy      = target.y - (points[i].y + t * sy);
        const double dist    = std::sqrt(dx * dx + dy * dy);
        if (dist < best_dist) {
            best_dist = dist;
            // 経路末端より先の目標は最終線分の延長上で測る
            if (i + 2 == points.size() && t_raw > 1.0) {
                t = t_raw;
            }
            best_arc = arc_start + t * length;
        }
        arc_start += length;
    }
    return std::max(best_arc - offset, 0.0);
}

std::vector<Point2D> to_world(const std::vector<Point2D>& points, const Pose2D& pose) {
    const double         c = std::cos(pose.yaw);
    const double         s = std::sin(pose.yaw);
    std::vector<Point2D> out;
    out.reserve(points.size());
    for (const Point2D& p : points) {
        out.push_back(Point2D{pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y});
    }
    return out;
}

nav_msgs::msg::Path make_path(
    const std::vector<Point2D>& points, const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp    = stamp;
    path.header.frame_id = frame_id;
    path.poses.reserve(points.size());
    double yaw = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i + 1 < points.size()) {
            yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header           = path.header;
        pose.pose.position.x  = points[i].x;
        pose.pose.position.y  = points[i].y;
        pose.pose.position.z  = 0.0;
        pose.pose.orientation = utils::yaw_to_quaternion(yaw);
        path.poses.push_back(pose);
    }
    return path;
}

}  // namespace vector_e2e_planner
