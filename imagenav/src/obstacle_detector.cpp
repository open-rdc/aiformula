#include "imagenav/obstacle_detector.hpp"

namespace imagenav{

ObstacleDetector::ObstacleDetector() 
{
}

// 障害物の座標推定
std::vector<cv::Point> ObstacleDetector::detectObstacle(const cv::Mat& img)
{
    cv::Mat img_yuv;
    cv::cvtColor(img, img_yuv, cv::COLOR_BGR2YUV);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    std::vector<cv::Mat> channels;
    cv::split(img_yuv, channels);
    // 輝度のチャンネルのみヒストグラム平滑化<-画像のコンストラストを改善
    clahe->apply(channels[0], channels[0]);
    cv::merge(channels, img_yuv);
    // gbrに戻す
    cv::Mat img_gbr, img_hsv;
    cv::cvtColor(img_yuv, img_gbr, cv::COLOR_YUV2BGR);
    // hsvに変換
    cv::cvtColor(img_gbr, img_hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_img = MaskObstacleImage(img_hsv);

    return EstimatePosition(mask_img);
}

cv::Mat ObstacleDetector::MaskObstacleImage(const cv::Mat& img)
{
    // 検出対象の色範囲を指定
    cv::Scalar LOW_COLOR1(0, 50, 50), HIGH_COLOR1(6, 255, 255);
    cv::Scalar LOW_COLOR2(174, 50, 50), HIGH_COLOR2(180, 255, 255);

    cv::Mat mask1, mask2, mask;
    cv::inRange(img, LOW_COLOR1, HIGH_COLOR1, mask1);
    // cv::inRange(img, LOW_COLOR2, HIGH_COLOR2, mask2);
    // cv::bitwise_and(mask1, mask2, mask);

    return mask1;
}

std::vector<cv::Point> ObstacleDetector::EstimatePosition(const cv::Mat& img)
{
    std::vector<cv::Point> obstacles_x;
    int max_white_x=-1;

    std::vector<int> white_pixel_counts(img.cols, 0);
    for(int y = 0; y < img.rows; ++y)
    {
        const uchar *pLine = img.ptr<uchar>(y);
        for(int x = 0; x < img.cols; ++x)
        {
            if(pLine[x] > 128)
            {
                white_pixel_counts[x]++;
            }
        }
    }

    for(int x = 0; x < img.cols; ++x)
    {
        if(white_pixel_counts[x] > white_pixel_counts[x - 1] && white_pixel_counts[x] >= 10)
        {
            max_white_x = x;
        }
    }

    obstacles_x.push_back(cv::Point{max_white_x, 0});
    return obstacles_x;
}

} // namespace