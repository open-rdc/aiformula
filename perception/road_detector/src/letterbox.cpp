#include "road_detector/letterbox.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace road_detector {

namespace {
constexpr double letterbox_value = 114.0;
constexpr int model_shape = 640;
constexpr int letterbox_stride = 32;

int round_half_to_even(const double value) { return static_cast<int>(std::nearbyint(value)); }
}  // namespace

LetterboxGeometry compute_letterbox_geometry(const cv::Size& image_size, const cv::Size& input_size)
{

    const double ratio = std::min(static_cast<double>(model_shape) / image_size.height, static_cast<double>(model_shape) / image_size.width);

    LetterboxGeometry geometry;
    geometry.content_width  = round_half_to_even(image_size.width * ratio);
    geometry.content_height = round_half_to_even(image_size.height * ratio);

    const double half_width  = ((model_shape - geometry.content_width) % letterbox_stride) / 2.0;
    const double half_height = ((model_shape - geometry.content_height) % letterbox_stride) / 2.0;
    geometry.pad_left        = round_half_to_even(half_width - 0.1);
    geometry.pad_top         = round_half_to_even(half_height - 0.1);
    const int pad_right      = round_half_to_even(half_width + 0.1);
    const int pad_bottom     = round_half_to_even(half_height + 0.1);

    // 組み立てた letterbox サイズが engine の入力と一致することを検証する。
    const cv::Size letterboxed(geometry.content_width + geometry.pad_left + pad_right, geometry.content_height + geometry.pad_top + pad_bottom);
    if (letterboxed != input_size) {
        throw std::runtime_error(
            "letterbox 後のサイズ " + std::to_string(letterboxed.width) + "x" +
            std::to_string(letterboxed.height) + " が engine の入力 " +
            std::to_string(input_size.width) + "x" + std::to_string(input_size.height) +
            " と一致しません(配信画像の解像度と engine の対応を確認してください)");
    }
    return geometry;
}

cv::Mat apply_letterbox(const cv::Mat& bgr_image, const cv::Size& input_size, const LetterboxGeometry& geometry)
{
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(geometry.content_width, geometry.content_height), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat padded(input_size, CV_8UC3, cv::Scalar::all(letterbox_value));
    resized.copyTo(padded(cv::Rect(geometry.pad_left, geometry.pad_top, geometry.content_width, geometry.content_height)));

    return padded;
}

void to_nchw(const cv::Mat& padded_bgr, std::vector<float>& blob)
{
    const int width  = padded_bgr.cols;
    const int height = padded_bgr.rows;
    blob.resize(static_cast<size_t>(3) * width * height);
    for (int channel = 0; channel < 3; ++channel) {
        for (int y = 0; y < height; ++y) {
            const uint8_t* row = padded_bgr.ptr<uint8_t>(y);
            float*         out = blob.data() + (static_cast<size_t>(channel) * height + y) * width;
            for (int x = 0; x < width; ++x) {
                out[x] = static_cast<float>(row[x * 3 + channel]) / 255.0F;
            }
        }
    }
}

cv::Mat to_lane_mask(const cv::Mat& confidence, const cv::Size& image_size, const LetterboxGeometry& geometry, const float threshold){
    cv::Mat binary;
    cv::compare(confidence, threshold, binary, cv::CMP_GE);

    const cv::Mat content = binary(cv::Rect(geometry.pad_left, geometry.pad_top, geometry.content_width, geometry.content_height));
    if (content.size() == image_size) {
        return content.clone();
    }

    cv::Mat mask;
    cv::resize(content, mask, image_size, 0.0, 0.0, cv::INTER_NEAREST);
    return mask;
}

}  // namespace road_detector
