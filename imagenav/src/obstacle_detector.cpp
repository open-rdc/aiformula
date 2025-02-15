#include "imagenav/obstacle_detector.hpp"

namespace imagenav{

ObstacleDetector::ObstacleDetector() 
{
}

// 障害物の座標推定
cv::Mat ObstacleDetector::detectObstacle(const cv::Mat& img)
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

    return detectObstacle(img_hsv);
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

        
    cv::imshow("obstacle image data", mask1);
    cv::waitKey(1);

    return mask1;
}

} // namespace