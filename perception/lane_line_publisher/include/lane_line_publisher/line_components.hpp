#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace lane_line_publisher
{

struct LineComponent
{
    cv::Mat mask;
    int pixel_count;
};

std::vector<LineComponent> split_line_components(
    const cv::Mat& skeleton_mask, int min_component_pixels);

}
