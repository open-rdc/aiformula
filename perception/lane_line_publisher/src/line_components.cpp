#include "lane_line_publisher/line_components.hpp"

#include <utility>

#include <opencv2/imgproc.hpp>

namespace lane_line_publisher
{

std::vector<LineComponent> split_line_components(
    const cv::Mat& skeleton_mask, const int min_component_pixels)
{
    cv::Mat labels, stats, centroids;
    const int num_labels = cv::connectedComponentsWithStats(
        skeleton_mask, labels, stats, centroids, 8, CV_32S);

    std::vector<LineComponent> components;
    for (int label = 1; label < num_labels; ++label) {
        const int pixel_count = stats.at<int>(label, cv::CC_STAT_AREA);
        if (pixel_count < min_component_pixels) {
            continue;
        }
        cv::Mat component_mask = cv::Mat::zeros(skeleton_mask.size(), CV_8UC1);
        component_mask.setTo(255, labels == label);
        components.push_back(LineComponent{std::move(component_mask), pixel_count});
    }
    return components;
}

}
