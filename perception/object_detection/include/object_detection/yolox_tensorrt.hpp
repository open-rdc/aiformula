#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace object_detection {

// 検出結果。class_id は学習時のクラス割り当て(0: パイロン, 1: 動的障害物)
struct Detection {
    cv::Rect2d box;
    int        class_id;
    float      score;
};

// YOLOX の TensorRT エンジンを読み込み、BGR画像から bbox を推論する。
// 学習側でモデル契約に従う ONNX を出力し、デプロイ先で engine に変換しておく。
// 詳細は weights/README.md を参照。
class YoloxTensorrt {
   public:
    YoloxTensorrt(const std::string& engine_path, double score_threshold, double nms_threshold);
    ~YoloxTensorrt();

    // 入力画像の画素座標系での検出結果を返す
    std::vector<Detection> detect(const cv::Mat& bgr_image);

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace object_detection
