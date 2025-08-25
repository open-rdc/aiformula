#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_line_publisher {

struct LaneLine {
    std::vector<cv::Point> pixels;
    
    void clear() {
        pixels.clear();
    }
};

struct LaneLines {
    LaneLine left;
    LaneLine right;
    LaneLine center;
    
    void clear() {
        left.clear();
        right.clear();
        center.clear();
    }
};

class LanePixelFinder {
public:
    explicit LanePixelFinder(int tolerance = 50) : tolerance_(tolerance) {}
    
    ~LanePixelFinder() = default;

    void searchMask(const cv::Mat& binary_mask, LaneLines& lane_lines) const;
    
    cv::Mat visualizeLanePixels(const cv::Mat& input_image, const LaneLines& lane_lines) const;
    
    int getTolerance() const {
        return tolerance_;
    }

private:
    int tolerance_=10;  // 探索許容範囲
};

}  // namespace lane_line_publisher