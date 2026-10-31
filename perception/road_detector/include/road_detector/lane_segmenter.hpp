#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>

#include "road_detector/visibility_control.h"

namespace road_detector {

// YOLOPv2 の TensorRT エンジンで白線の確信度マップ(CV_32FC1, 0..1)を推論する(非スレッド安全)
class ROAD_DETECTOR_PUBLIC LaneSegmenter {
   public:
    // engine ファイルを開けなければ std::runtime_error を投げる
    explicit LaneSegmenter(const std::string& engine_path);
    ~LaneSegmenter();

    // engine が期待する入力サイズ
    cv::Size input_size() const;

    // padded_bgr は input_size() と同サイズの CV_8UC3。戻り値は次回の呼び出しで上書きされる内部バッファ
    cv::Mat infer(const cv::Mat& padded_bgr);

    // 直近の infer() が空 Mat を返した理由。infer() の呼び出しごとにクリアされる
    const std::string& last_error() const;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace road_detector
