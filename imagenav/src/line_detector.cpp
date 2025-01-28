#include "imagenav/line_detector.hpp"

#include "cmath"

namespace imagenav{

LineDetector::LineDetector()
{
}

cv::Mat LineDetector::detectLine(const cv::Mat& cv_img)
{
    cv::Mat gray_img, gaussian_img, edge_img;
    cv::cvtColor(cv_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray_img, gray_img);
    cv::GaussianBlur(gray_img, gaussian_img, cv::Size(ksize, ksize), 5);
    cv::Canny(gaussian_img, edge_img, min_th, max_th, 3);

    std::vector <cv::Vec4i> lines;
    cv::HoughLinesP(edge_img, lines, 1, CV_PI / 180, threshold, 0.2, 5);

    return draw_lines(cv_img, lines);
}

cv::Mat LineDetector::draw_lines(const cv::Mat& cv_img ,const std::vector<cv::Vec4i>& line)
{
    for(size_t i = 0; i < line.size(); i++)
    {
        if(line[i][0] != line[i][2])
        {
            double slope = (line[i][3] - line[i][1])/(line[i][2] - line[i][0]);
            // 傾きから角度を求める
            double theta = std::atan(slope);
            if(slope > 0)
            {
                if(line[i][2] > line[i][0])
                {
                    int x = line[i][2] + length*std::cos(theta);
                    int y = line[i][3] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][0], line[i][1]), cv::Scalar(0, 0, 255), thickness);
                }else
                {
                    int x = line[i][0] + length*std::cos(theta);
                    int y = line[i][1] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][2], line[i][3]), cv::Scalar(0, 0, 255), thickness);
                }
            }else{
                if(line[i][2] > line[i][0])
                {
                    int x = line[i][0] + length*std::cos(theta);
                    int y = line[i][1] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][2], line[i][3]), cv::Scalar(0, 0, 255), thickness);
                }else{
                    int x = line[i][2] + length*std::cos(theta);
                    int y = line[i][3] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][0], line[i][1]), cv::Scalar(0, 0, 255), thickness);
                }
            }
        }
    }
    return cv_img;
}

// std::vector<Position> LineDetector::LineToPoint(const std::vector<cv::Vec4i>& line, double num_point)
// {
//     for(size_t i=0; i < xs.size(); ++i)
//     {

//     }
// }


} // namespace