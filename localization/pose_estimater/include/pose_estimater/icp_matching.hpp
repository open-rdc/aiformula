#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace pose_estimater
{

struct IcpConfig
{
    int max_iterations;
    double max_correspondence_distance;
    double convergence_translation_epsilon;
    std::size_t min_correspondences;
    double max_mean_error;
};

struct IcpMapPoint
{
    Eigen::Vector2d position;
    Eigen::Vector2d normal;
};

struct IcpResult
{
    bool converged = false;
    Eigen::Vector2d translation = Eigen::Vector2d::Zero();
    std::size_t correspondences = 0U;
    double mean_error = 0.0;
    Eigen::Matrix2d normal_matrix = Eigen::Matrix2d::Zero();
};

class IcpTargetMap
{
public:
    explicit IcpTargetMap(std::vector<IcpMapPoint> points);

    bool empty() const;

    bool nearest(
        const Eigen::Vector2d& query,
        double max_distance_sq,
        std::size_t& nearest_index) const;

    const IcpMapPoint& point(std::size_t index) const;

private:
    struct KdNode
    {
        std::size_t point_index;
        int left;
        int right;
        int axis;
    };

    int build_tree(std::vector<std::size_t>& indices, std::size_t begin, std::size_t end, int depth);
    void nearest_recursive(
        int node_index,
        const Eigen::Vector2d& query,
        std::size_t& nearest_index,
        double& nearest_distance_sq,
        bool& found) const;

    std::vector<IcpMapPoint> points_;
    std::vector<KdNode> nodes_;
    int root_index_;
};

class IcpMatcher
{
public:
    explicit IcpMatcher(const IcpConfig& config);

    IcpResult align_translation_only(
        const std::vector<Eigen::Vector2d>& source_points,
        const IcpTargetMap& target_map) const;

private:
    IcpConfig config_;
};

}
