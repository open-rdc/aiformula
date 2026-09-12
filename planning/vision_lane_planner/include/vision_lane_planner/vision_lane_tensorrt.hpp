#pragma once

#include "vision_lane_planner/lane_path_builder.hpp"
#include "vision_lane_planner/visibility_control.h"

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace vision_lane_planner {

// VisionPlanner の TensorRT エンジンを読み込み、letterbox 済み画像から
// スロットごとの行アンカー予測を返す。
// 学習側で argmax ヘッドの ONNX を出力し、デプロイ先で engine に変換しておく。
// 詳細は weights/README.md を参照。
class VisionLaneTensorrt {
   public:
    VISION_LANE_PLANNER_PUBLIC
    explicit VisionLaneTensorrt(const std::string& engine_path);

    VISION_LANE_PLANNER_PUBLIC
    ~VisionLaneTensorrt();

    // padded は 384x640 の CV_8UC3。失敗時は valid = false を返し、理由は last_error() に入る
    VISION_LANE_PLANNER_PUBLIC
    SlotPrediction infer(const cv::Mat& padded);

    VISION_LANE_PLANNER_PUBLIC
    const std::string& last_error() const;

    // エンジンのデシリアライズ等で TensorRT ロガーが出した直近の警告以下のメッセージ。
    // 別デバイスで焼かれた plan file やバージョン差異の互換警告はここにしか残らない
    VISION_LANE_PLANNER_PUBLIC
    const std::string& last_warning() const;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace vision_lane_planner
