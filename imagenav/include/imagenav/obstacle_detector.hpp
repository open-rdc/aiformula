#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

namespace imagenav{

class ObstacleDetector {

typedef struct{
    double x;
    double y;
}Obstacle;

public:
    ObstacleDetector();
    cv::Point detectPotition(const cv::Mat& img);

private:
    cv::Mat toBEV(const cv::Mat& img);
    cv::Point detectObstacle(const cv::Mat& img);
};

} // namespace