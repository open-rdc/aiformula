#pragma once

#include <vector>

namespace local_planner::frenet
{

struct Point2D { double x; double y; };
struct Pose2D { double x; double y; double yaw; };
struct FrenetPose { double s; double d; double yaw; };  // yaw は射影点での参照線の向き

// 折れ線の参照線。弧長 s と左向き横位置 d の Frenet 座標と直交座標を相互に変換する
class ReferencePath
{
public:
    void set_points(const std::vector<Point2D> & points);

    std::size_t size() const;
    double length() const;
    FrenetPose project(const Point2D & point) const;
    Pose2D pose_at(double s) const;
    double curvature_at(double s, double h) const;
    std::vector<Pose2D> to_cartesian(
        const std::vector<double> & s_points, const std::vector<double> & offsets) const;

private:
    struct Sample { double s; double x; double y; };

    std::vector<Sample> samples_;
};

}
