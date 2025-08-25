#include "lane_line_publisher/lane_pixel_finder.hpp"
#include <algorithm>

namespace lane_line_publisher {

void LanePixelFinder::searchMask(const cv::Mat& binary_mask, LaneLines& lane_lines) const {
    const auto cols = binary_mask.cols;
    const auto rows = binary_mask.rows;

    // Initializations.
    const int top = 0;
    const int bottom = rows - 1;
    int left = 0;
    int right = cols - 1;
    int center = (left + right) / 2;

    for (int row = bottom; row >= top; --row) {
        const auto row_ptr = binary_mask.ptr<uchar>(row);
        if (row_ptr[center]) break;

        bool found_left = false;
        const auto left_bound = std::max(0, left - tolerance_);
        for (int col = center; col >= left_bound; --col) {
            if (row_ptr[col]) {
                left = col;
                found_left = true;
                break;
            }
        }

        bool found_right = false;
        const auto right_bound = std::min(cols - 1, right + tolerance_);
        for (int col = center; col <= right_bound; ++col) {
            if (row_ptr[col]) {
                right = col;
                found_right = true;
                break;
            }
        }

        center = (left + right) / 2;

        if (found_left) lane_lines.left.pixels.emplace_back(left, row);
        if (found_right) lane_lines.right.pixels.emplace_back(right, row);
        if (found_left && found_right) lane_lines.center.pixels.emplace_back(center, row);
    }
}

cv::Mat LanePixelFinder::visualizeLanePixels(const cv::Mat& input_image, const LaneLines& lane_lines) const {
    cv::Mat visualization;
    if (input_image.channels() == 1) {
        cv::cvtColor(input_image, visualization, cv::COLOR_GRAY2BGR);
    } else {
        visualization = input_image.clone();
    }
    
    for (const auto& pixel : lane_lines.left.pixels) {
        cv::circle(visualization, pixel, 2, cv::Scalar(255, 0, 0), -1);
    }
    
    for (const auto& pixel : lane_lines.right.pixels) {
        cv::circle(visualization, pixel, 2, cv::Scalar(0, 0, 255), -1);
    }
    
    int overlap_count = 0;
    for (const auto& left_pixel : lane_lines.left.pixels) {
        for (const auto& right_pixel : lane_lines.right.pixels) {
            if (left_pixel.x == right_pixel.x && left_pixel.y == right_pixel.y) {
                cv::circle(visualization, left_pixel, 3, cv::Scalar(0, 255, 255), -1);
                overlap_count++;
            }
        }
    }
    
    // Add text information
    std::string left_count = "Left pixels: " + std::to_string(lane_lines.left.pixels.size());
    std::string right_count = "Right pixels: " + std::to_string(lane_lines.right.pixels.size());
    std::string overlap_text = "Overlapping pixels: " + std::to_string(overlap_count);
    
    cv::putText(visualization, left_count, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
    cv::putText(visualization, right_count, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    cv::putText(visualization, overlap_text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    
    // Legend
    cv::putText(visualization, "Blue: Left lane", cv::Point(10, visualization.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    cv::putText(visualization, "Red: Right lane", cv::Point(10, visualization.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::putText(visualization, "Yellow: Overlap", cv::Point(10, visualization.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    
    return visualization;
}

}  // namespace lane_line_publisher