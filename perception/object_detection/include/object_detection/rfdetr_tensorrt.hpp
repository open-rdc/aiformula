#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace object_detection {

struct Detection {
    cv::Rect2d box;
    int        class_id;
    float      score;
};

// RF-DETR の TensorRT エンジンを読み込み、BGR画像から bbox を推論する。
// 学習側で出力した ONNX をデプロイ先で engine に変換しておく。詳細は weights/README.md を参照。
class RfdetrTensorrt {
   public:
    RfdetrTensorrt(const std::string& engine_path, double score_threshold);
    ~RfdetrTensorrt();

    // 入力画像の画素座標系での検出結果を返す
    std::vector<Detection> detect(const cv::Mat& bgr_image);

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace object_detection
