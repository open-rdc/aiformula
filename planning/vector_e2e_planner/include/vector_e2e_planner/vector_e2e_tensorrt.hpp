#pragma once

#include "vector_e2e_planner/path_builder.hpp"
#include "vector_e2e_planner/visibility_control.h"

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace vector_e2e_planner {

// vector_planner の TensorRT エンジンを読み込み、letterbox 済み BGR 画像から
// 経路・走行帯・分岐存在・信号の出力を返す。
// 学習側で ONNX を出力し、デプロイ先で engine に変換しておく。
// 詳細は weights/README.md を参照。
class VectorE2eTensorrt {
   public:
    VECTOR_E2E_PLANNER_PUBLIC
    explicit VectorE2eTensorrt(const std::string& engine_path);

    VECTOR_E2E_PLANNER_PUBLIC
    ~VectorE2eTensorrt();

    // padded は 384x640 の CV_8UC3（BGR）。失敗時は valid = false を返し、理由は last_error() に入る
    VECTOR_E2E_PLANNER_PUBLIC
    ModelOutputs infer(const cv::Mat& padded);

    VECTOR_E2E_PLANNER_PUBLIC
    const std::string& last_error() const;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace vector_e2e_planner
