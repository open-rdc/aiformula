#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace vectormap_localization
{

struct IcpConfig
{
    int max_iterations;
    double max_correspondence_distance;
    double convergence_translation_epsilon;
    std::size_t min_correspondences;
};

struct IcpResult
{
    bool converged;
    Eigen::Vector2d translation;
    std::size_t correspondences;
    double mean_error;
};

class IcpTargetMap
{
public:
    explicit IcpTargetMap(std::vector<Eigen::Vector2d> points);

    bool empty() const;
    std::size_t size() const;
    const std::vector<Eigen::Vector2d>& points() const;

    bool nearest(
        const Eigen::Vector2d& query,
        double max_distance_sq,
        Eigen::Vector2d& nearest_point,
        double& nearest_distance_sq) const;

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
        double max_distance_sq,
        Eigen::Vector2d& nearest_point,
        double& nearest_distance_sq,
        bool& found) const;

    std::vector<Eigen::Vector2d> points_;
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

}  // namespace vectormap_localization
