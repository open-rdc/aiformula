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
    std::vector<cv::Point> SlideWindowMethod(const cv::Mat& img, const int start_x, const int line);
    std::vector<cv::Point2f> BEVtoScreen(const std::vector<cv::Point2f>& points);
    cv::Mat WindowVisualizar(const cv::Mat& img, const std::vector<cv::Point2f>& points, const int line, const cv::Scalar color);
    cv::Mat PointVisualizar(const cv::Mat& img, const std::vector<cv::Point2f>& points, const cv::Scalar color);
    
private:
    cv::Mat draw_lines(const cv::Mat& cv_img, const std::vector<cv::Vec4i>& line);
    cv::Mat toBEV(const cv::Mat& img);
    bool JunctionDetectWindow(const cv::Mat& img, const cv::Point pos, const int delta_x);

    std::vector<int> prev_start_xs={100, 400};
    std::vector<int> window_width={100, 100};

    std::vector<std::vector<cv::Point>> prev_window_position = {
        {cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0)},
        {cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0),cv::Point(0,0)}
    };

    cv::Mat inv_trans_mat;

    int find_window_x=0;
    bool junction_flag_=true;
    bool line_start_flag=true;

    const int window_height=40;
};

} // namespace