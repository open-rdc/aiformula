#include "object_detection/pylon_mask.hpp"

#include <opencv2/imgproc.hpp>

namespace object_detection {

namespace {
constexpr int   pylon_class_id        = 0;
constexpr float pylon_score_threshold = 0.3F;
}  // namespace

cv::Mat draw_pylon_mask(const std::vector<Detection>& detections, const int height, const int width) {
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
    for (const auto& detection : detections) {
        if (detection.class_id != pylon_class_id || detection.score < pylon_score_threshold) {
            continue;
        }
        cv::rectangle(mask, cv::Rect(detection.box), cv::Scalar(255), cv::FILLED);
    }
    return mask;
}

}  // namespace object_detection
