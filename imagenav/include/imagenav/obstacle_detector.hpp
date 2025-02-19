#pragma once

#include <cv_bridge/cv_bridge.h>

namespace imagenav{

class ObstacleDetector {

typedef struct{
    double x;
    double y;
}Obstacle;

public:
    ObstacleDetector();
    std::vector<cv::Point> detectObstacle(const cv::Mat& img);

private:
    cv::Mat MaskObstacleImage(const cv::Mat& img);
    std::vector<cv::Point> EstimatePosition(const cv::Mat& img);
};

} // namespace