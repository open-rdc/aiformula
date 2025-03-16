#include "imagenav/obstacle_detector.hpp"

namespace imagenav{

ObstacleDetector::ObstacleDetector() 
{
}

// 障害物の座標推定
std::vector<cv::Point> ObstacleDetector::detectObstacle(const cv::Mat& img, const cv::Mat& depth_img)
{
    cv::Mat img_copy = img.clone();
    cv::Mat mask_img = ObstacleMaskImage(img_copy);

    return EstimatePosition(mask_img, depth_img);;
}

cv::Mat ObstacleDetector::ObstacleMaskImage(const cv::Mat& img)
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

    cv::Scalar LOW_COLOR1(0, 50, 50), HIGH_COLOR1(0, 255, 255);
    cv::Scalar LOW_COLOR2(174, 50, 50), HIGH_COLOR2(180, 255, 255);

    cv::Mat mask_img, mask1, mask2;
    cv::inRange(img_hsv, LOW_COLOR1, HIGH_COLOR1, mask1);
    cv::inRange(img_hsv, LOW_COLOR2, HIGH_COLOR2, mask2);

    cv::bitwise_or(mask1, mask2, mask_img);

    return mask_img;
}

std::vector<cv::Point> ObstacleDetector::EstimatePosition(const cv::Mat& img, const cv::Mat& depth_img)
{
    cv::Mat depth_img_copy = depth_img.clone();
    std::vector<cv::Point> obstacle_positions;

    // 障害物の輪郭を検出
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 2) continue; // 小さすぎる輪郭はノイズとして無視

        // 輪郭の中心座標を求める
        cv::Moments mu = cv::moments(contour);
        if (mu.m00 == 0) continue;

        int cx = static_cast<int>(mu.m10 / mu.m00);
        int cy = static_cast<int>(mu.m01 / mu.m00);

        float depth_value = depth_img.at<float>(cy, cx);

        // 深度値が有効かチェック
        if (std::isnan(depth_value) || depth_value <= 0.0f || depth_value > 5000.0f) {
            continue;
        }

        obstacle_positions.emplace_back(cx, cy);
    }

    return obstacle_positions;
}

} // namespace