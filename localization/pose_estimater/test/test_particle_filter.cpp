#include <gtest/gtest.h>

#include <vector>

#include "pose_estimater/particle_filter.hpp"

using pose_estimater::PfMapPoint;
using pose_estimater::PfTargetMap;

TEST(PfTargetMapTest, NearestFindsClosestPointWithinRadius)
{
    const std::vector<PfMapPoint> points{
        PfMapPoint{Eigen::Vector2d(0.0, 0.0)},
        PfMapPoint{Eigen::Vector2d(5.0, 0.0)},
        PfMapPoint{Eigen::Vector2d(10.0, 0.0)},
    };
    const PfTargetMap map(points);

    std::size_t nearest_index = 0U;
    EXPECT_TRUE(map.nearest(Eigen::Vector2d(4.5, 0.1), 4.0, nearest_index));
    EXPECT_EQ(nearest_index, 1U);

    EXPECT_FALSE(map.nearest(Eigen::Vector2d(100.0, 100.0), 4.0, nearest_index));
}

TEST(PfTargetMapTest, EmptyMapReportsEmpty)
{
    const PfTargetMap map(std::vector<PfMapPoint>{});

    EXPECT_TRUE(map.empty());
    std::size_t nearest_index = 0U;
    EXPECT_FALSE(map.nearest(Eigen::Vector2d(0.0, 0.0), 1.0, nearest_index));
}
