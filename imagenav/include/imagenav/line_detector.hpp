#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

namespace imagenav{

class LineDetector {

public:
    LineDetector();
    cv::Mat detectLine(const cv::Mat& cv_img);
    std::vector<int> estimateLinePosition(const cv::Mat& img);
    std::vector<cv::Point> SlideWindowMethod(const cv::Mat& img, const int start_x);
    cv::Mat WindowVisualizar(cv::Mat& img, const std::vector<cv::Point>& points, const int line);
    cv::Mat PointVisualizar(cv::Mat& img, const std::vector<cv::Point>& points);

private:
    cv::Mat draw_lines(const cv::Mat& cv_img, const std::vector<cv::Vec4i>& line);
    std::vector<double> Spline(const std::vector<double> xs, const std::vector<double> ys, double num_point);
    cv::Mat toBEV(const cv::Mat& img);
    std::vector<int> prev_start_xs={100, 350};
    std::vector<int> window_width={100, 100};
};

} // namespace