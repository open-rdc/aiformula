#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "road_detector/visibility_control.h"

namespace road_detector {

// letterbox の幾何。元画像サイズと engine 入力サイズから一意に決まる
struct LetterboxGeometry {
    int content_width  = 0;  // 縮小後の有効領域の幅
    int content_height = 0;  // 縮小後の有効領域の高さ
    int pad_left       = 0;
    int pad_top        = 0;
};

// 元画像サイズと engine 入力サイズから letterbox の幾何を求める。
// 現行 Python 実装 letterbox(new_shape=(640,640), auto=True, stride=32) と同じ順方向の式
// (MODEL_INPUT_SHAPE=640 起点で ratio を求め、stride で余白を丸める)で計算し、
// 組み立てた結果が input_size と一致するかを検証する。一致しなければ例外を投げる
// (engine の入力サイズから ratio を逆算する方式は短辺の余白が 0 になる形状で非等価になるため使わない)
ROAD_DETECTOR_PUBLIC LetterboxGeometry compute_letterbox_geometry(
    const cv::Size& image_size, const cv::Size& input_size);

// BGR8 画像を engine 入力サイズへレターボックスする(余白は 114 で埋める)
ROAD_DETECTOR_PUBLIC cv::Mat apply_letterbox(
    const cv::Mat& bgr_image, const cv::Size& input_size, const LetterboxGeometry& geometry);

// letterbox 済み BGR8 画像を /255.0 して NCHW float32 に並べ替える(BGR 順のまま)
ROAD_DETECTOR_PUBLIC void to_nchw(const cv::Mat& padded_bgr, std::vector<float>& blob);

// 確信度マップ(CV_32FC1, engine 入力と同解像度)を二値マスクにし、元画像サイズへ戻す
ROAD_DETECTOR_PUBLIC cv::Mat to_lane_mask(
    const cv::Mat& confidence, const cv::Size& image_size, const LetterboxGeometry& geometry,
    float threshold);

}  // namespace road_detector
