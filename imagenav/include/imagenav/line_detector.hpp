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

private:
    cv::Mat draw_lines(const cv::Mat& cv_img, const std::vector<cv::Vec4i>& line);
    std::vector<double> Spline(const std::vector<double> xs, const std::vector<double> ys, double num_point);

    const int threshold=50; // ハフ変換しきい値
    const int ksize=9; // ガウシアンカーネルサイズ
    const int min_th=50;
    const int max_th=150;

    const int length = 100;
    const int thickness = 2;  
};

} // namespace