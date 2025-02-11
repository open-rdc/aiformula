#include "imagenav/obstacle_detector.hpp"

namespace imagenav{

ObstacleDetector::ObstacleDetector() 
{
}

// 障害物の座標推定
cv::Point ObstacleDetector::detectPotition(const cv::Mat& img)
{
    cv::Mat birdview_img = toBEV(img);

    // 確認用
    // cv::imshow("BEV image data", birdview_img);
    // cv::waitKey(1);

    return detectObstacle(birdview_img);
}

// 鳥瞰図変換
cv::Mat ObstacleDetector::toBEV(const cv::Mat& img)
{
    // 俯瞰画像の変換座標
    cv::Point2f src_points[4] = { {-950, 1080}, {-300, 600}, {2220, 600}, {2870, 1080} };
    cv::Point2f dst_points[4] = { {0, 1920}, {0, 0}, {1920, 0}, {1920, 1920} };

    cv::Mat trans_mat = cv::getPerspectiveTransform(src_points, dst_points);
    cv::Mat out_img;

    // trans_matを用いた変換
    cv::warpPerspective(img, out_img, trans_mat, img.size());

    return out_img;
}

cv::Point ObstacleDetector::detectObstacle(const cv::Mat& img)
{
    cv::Mat img_yuv;
    cv::cvtColor(img, img_yuv, cv::COLOR_BGR2YUV);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    std::vector<cv::Mat> channels;
    cv::split(img_yuv, channels);
    // 輝度のチャンネルのみヒストグラム平滑化
    clahe->apply(channels[0], channels[0]);
    cv::merge(channels, img_yuv);
    // gbrに戻す
    cv::Mat img_gbr, img_hsv;
    cv::cvtColor(img_yuv, img_gbr, cv::COLOR_YUV2BGR);
    // hsvに変換
    cv::cvtColor(img_gbr, img_hsv, cv::COLOR_BGR2HSV);

    // 検出対象の色範囲を指定
    cv::Scalar LOW_COLOR1(0, 50, 50), HIGH_COLOR1(6, 255, 255);
    cv::Scalar LOW_COLOR2(174, 50, 50), HIGH_COLOR2(180, 255, 255);

    cv::Mat mask1, mask2, mask;
    cv::inRange(img_hsv, LOW_COLOR1, HIGH_COLOR1, mask1);
    cv::inRange(img_hsv, LOW_COLOR2, HIGH_COLOR2, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 検出対象の座標取得
    if (!contours.empty())
    {
        auto largest = std::max_element(contours.begin(), contours.end(),
                                   [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                       return cv::contourArea(a) < cv::contourArea(b);
                                   });
        cv::Moments M = cv::moments(*largest);
        if (M.m00 > 0)
        {
            return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
        }
    }

    return cv::Point(-1, -1);

}

}
