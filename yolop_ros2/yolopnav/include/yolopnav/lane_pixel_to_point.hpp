#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

namespace yolopnav {

struct CameraIntrinsics {
    static constexpr double fx = 184.919;       // Focal length in x direction (pixels) - scaled from 246.558 × (480/640)
    static constexpr double fy = 205.324;       // Focal length in y direction (pixels) - scaled from 246.389 × (300/360)
    static constexpr double cx = 238.759;       // Principal point x coordinate (pixels) - scaled from 318.345 × (480/640)
    static constexpr double cy = 155.492;       // Principal point y coordinate (pixels) - scaled from 186.590 × (300/360)
    
    // Image size
    static constexpr int width = 480;
    static constexpr int height = 300;
    
    static constexpr double camera_height = 0.54;
};

class LanePixelToPoint {
public:
    LanePixelToPoint();
    ~LanePixelToPoint() = default;

    Eigen::Vector3d pixelToRobotPoint(const cv::Point& pixel) const;
    std::vector<Eigen::Vector3d> pixelsToRobotPoints(const std::vector<cv::Point>& pixels) const;

private:
    cv::Mat inv_camera_matrix_;  // Pre-computed inverse camera matrix
};

}  // namespace yolopnav