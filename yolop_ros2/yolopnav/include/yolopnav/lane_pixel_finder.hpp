#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

namespace yolopnav {

struct LaneLine {
    std::vector<cv::Point> pixels;                    // Screen pixel coordinates 
    std::vector<Eigen::Vector3d> points;              // Robot coordinate system points (matching aiformula)
    
    void clear() {
        pixels.clear();
        points.clear();
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

}  // namespace yolopnav