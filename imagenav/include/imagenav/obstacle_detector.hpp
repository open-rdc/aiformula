#pragma once

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include<numeric>

namespace imagenav{

class ObstacleDetector {

public:
    ObstacleDetector();
    cv::Mat DebugImage(const cv::Mat& img);
    std::vector<cv::Point> detectObstacle(const cv::Mat& img, const cv::Mat& depth_img);

private:
    cv::Mat ObstacleMaskImage(const cv::Mat& img);
    std::vector<cv::Point> EstimatePosition(const cv::Mat& img, const cv::Mat& depth_img);
};

} // namespace