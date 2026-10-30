#pragma once

#include "object_detection/yolox_tensorrt.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace object_detection {

// pylon かつ score >= 0.3 の枠を 255 で塗った CV_8UC1 のマスク。
// e2e_planner の学習データ（e2e_planner/scripts/util/pylon_mask.py の draw_pylon_mask）と同じ規則にしてある。
// どちらかを変えたら両方を直すこと
cv::Mat draw_pylon_mask(const std::vector<Detection>& detections, int height, int width);

}  // namespace object_detection
