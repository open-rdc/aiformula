#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_line_publisher/line_components.hpp"

namespace
{

cv::Mat make_mask(int rows, int cols, const std::vector<cv::Point>& on_pixels)
{
    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (const auto& p : on_pixels) {
        mask.at<uint8_t>(p.y, p.x) = 255;
    }
    return mask;
}

}  // namespace

TEST(SplitLineComponents, DropsComponentsBelowMinPixelCount)
{
    const auto mask = make_mask(50, 50, {cv::Point(10, 10)});  // pixel_count == 1
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    EXPECT_TRUE(components.empty());
}

TEST(SplitLineComponents, KeepsComponentsAtOrAboveMinPixelCount)
{
    const auto mask = make_mask(
        50, 50, {cv::Point(10, 10), cv::Point(11, 10), cv::Point(12, 10),
                 cv::Point(13, 10), cv::Point(14, 10)});  // pixel_count == 5
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    ASSERT_EQ(components.size(), 1U);
    EXPECT_EQ(components[0].pixel_count, 5);
}

TEST(SplitLineComponents, SeparatesTwoDisconnectedComponents)
{
    const auto mask = make_mask(
        50, 50,
        {cv::Point(10, 10), cv::Point(11, 10), cv::Point(12, 10), cv::Point(13, 10), cv::Point(14, 10),
         cv::Point(30, 30), cv::Point(31, 30), cv::Point(32, 30), cv::Point(33, 30), cv::Point(34, 30)});
    const auto components = lane_line_publisher::split_line_components(mask, 5);
    ASSERT_EQ(components.size(), 2U);
    EXPECT_EQ(components[0].pixel_count, 5);
    EXPECT_EQ(components[1].pixel_count, 5);
    EXPECT_EQ(cv::countNonZero(components[0].mask), 5);
    EXPECT_EQ(cv::countNonZero(components[1].mask), 5);
}
