#include "local_planner/frenet/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner::frenet
{

void ReferencePath::set_points(const std::vector<Point2D> & points)
{
    samples_.clear();
    double s = 0.0;
    for (const auto & point : points) {
        if (!samples_.empty()) {
            const double ds = std::hypot(point.x - samples_.back().x, point.y - samples_.back().y);
            if (ds < 1.0e-6) {
                continue;
            }
            s += ds;
        }
        samples_.push_back(Sample{s, point.x, point.y});
    }
}

std::size_t ReferencePath::size() const
{
    return samples_.size();
}

double ReferencePath::length() const
{
    return samples_.back().s;
}

FrenetPose ReferencePath::project(const Point2D & point) const
{
    double best_distance_sq = std::numeric_limits<double>::max();
    FrenetPose best{};
    for (std::size_t i = 1U; i < samples_.size(); ++i) {
        const auto & a = samples_[i - 1U];
        const auto & b = samples_[i];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double t = std::clamp(((point.x - a.x) * vx + (point.y - a.y) * vy) / (vx * vx + vy * vy), 0.0, 1.0);
        const double dx = point.x - (a.x + t * vx);
        const double dy = point.y - (a.y + t * vy);
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            const double yaw = std::atan2(vy, vx);
            best = FrenetPose{a.s + t * (b.s - a.s), -std::sin(yaw) * dx + std::cos(yaw) * dy, yaw};
        }
    }
    return best;
}

Pose2D ReferencePath::pose_at(const double s) const
{
    const double clamped_s = std::clamp(s, 0.0, length());
    const auto end = std::upper_bound(
        samples_.begin() + 1, samples_.end() - 1, clamped_s,
        [](const double value, const Sample & sample) { return value < sample.s; });
    const auto & start = *(end - 1);
    const double ratio = (clamped_s - start.s) / (end->s - start.s);
    return Pose2D{
        start.x + ratio * (end->x - start.x),
        start.y + ratio * (end->y - start.y),
        std::atan2(end->y - start.y, end->x - start.x)};
}

double ReferencePath::curvature_at(const double s, const double h) const
{
    return std::remainder(pose_at(s + h).yaw - pose_at(s - h).yaw, 2.0 * M_PI) / (2.0 * h);
}

std::vector<Pose2D> ReferencePath::to_cartesian(
    const std::vector<double> & s_points, const std::vector<double> & offsets) const
{
    std::vector<Pose2D> points;
    for (std::size_t i = 0U; i < s_points.size(); ++i) {
        const auto base = pose_at(s_points[i]);
        points.push_back(Pose2D{
            base.x - std::sin(base.yaw) * offsets[i], base.y + std::cos(base.yaw) * offsets[i], base.yaw});
    }
    // 向きは隣接点を結ぶ方向、終端は直前の区間の向き
    for (std::size_t i = 1U; i < points.size(); ++i) {
        points[i - 1U].yaw = std::atan2(points[i].y - points[i - 1U].y, points[i].x - points[i - 1U].x);
        points[i].yaw = points[i - 1U].yaw;
    }
    return points;
}

}
