#include "object_detection/yolox_tensorrt.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace object_detection {

namespace {

// YOLOX head の出力ストライド(固定)
constexpr int strides[] = {8, 16, 32};
// YOLOX ValTransform のレターボックス余白画素値
constexpr double letterbox_value = 114.0;

class TensorrtLogger : public nvinfer1::ILogger {
   public:
    void log(const Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            last_message_ = msg;
        }
    }

    const std::string& last_message() const { return last_message_; }

   private:
    std::string last_message_;
};

// YOLOX の前処理: アスペクト比を保って縮小し左上詰めで 114 埋め、正規化はしない
double letterbox(const cv::Mat& bgr_image, const int width, const int height, cv::Mat& padded) {
    const double ratio = std::min(static_cast<double>(height) / bgr_image.rows, static_cast<double>(width) / bgr_image.cols);
    const int resized_width  = static_cast<int>(bgr_image.cols * ratio);
    const int resized_height = static_cast<int>(bgr_image.rows * ratio);

    padded = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(letterbox_value));
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
    resized.copyTo(padded(cv::Rect(0, 0, resized_width, resized_height)));
    return ratio;
}

// BGR のまま 0-255 を維持して NCHW に並べ替える
void to_nchw(const cv::Mat& padded, std::vector<float>& blob) {
    const int width  = padded.cols;
    const int height = padded.rows;
    blob.resize(static_cast<size_t>(3) * width * height);
    for (int channel = 0; channel < 3; ++channel) {
        for (int y = 0; y < height; ++y) {
            const uint8_t* row = padded.ptr<uint8_t>(y);
            float*         out = blob.data() + (static_cast<size_t>(channel) * height + y) * width;
            for (int x = 0; x < width; ++x) {
                out[x] = static_cast<float>(row[x * 3 + channel]);
            }
        }
    }
}

// YOLOX head の生出力(格子オフセット)を画素座標の bbox に戻す
void decode(
    const float*             output,
    const int                num_attributes,
    const int                width,
    const int                height,
    const double             score_threshold,
    std::vector<cv::Rect2d>& boxes,
    std::vector<float>&      scores,
    std::vector<int>&        class_ids) {
    const int num_classes = num_attributes - 5;
    int       index       = 0;
    for (const int stride : strides) {
        const int grid_width  = width / stride;
        const int grid_height = height / stride;
        for (int grid_y = 0; grid_y < grid_height; ++grid_y) {
            for (int grid_x = 0; grid_x < grid_width; ++grid_x, ++index) {
                const float* feature     = output + static_cast<size_t>(index) * num_attributes;
                float        class_score = 0.0F;
                int          class_id    = 0;
                for (int c = 0; c < num_classes; ++c) {
                    if (feature[5 + c] > class_score) {
                        class_score = feature[5 + c];
                        class_id    = c;
                    }
                }
                const float score = feature[4] * class_score;
                if (score < score_threshold) {
                    continue;
                }
                const double center_x   = (feature[0] + grid_x) * stride;
                const double center_y   = (feature[1] + grid_y) * stride;
                const double box_width  = std::exp(feature[2]) * stride;
                const double box_height = std::exp(feature[3]) * stride;
                boxes.emplace_back(
                    center_x - box_width * 0.5, center_y - box_height * 0.5, box_width, box_height);
                scores.push_back(score);
                class_ids.push_back(class_id);
            }
        }
    }
}

}  // namespace

class YoloxTensorrt::Implementation {
   public:
    TensorrtLogger                               logger;
    std::unique_ptr<nvinfer1::IRuntime>          runtime;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t                                 stream = nullptr;

    std::string input_name;
    std::string output_name;
    int         input_width    = 0;
    int         input_height   = 0;
    int         num_anchors    = 0;
    int         num_attributes = 0;

    void*              input_device  = nullptr;
    void*              output_device = nullptr;
    std::vector<float> input_host;
    std::vector<float> output_host;

    double score_threshold = 0.0;
    double nms_threshold   = 0.0;

    ~Implementation() {
        if (input_device != nullptr) {
            cudaFree(input_device);
        }
        if (output_device != nullptr) {
            cudaFree(output_device);
        }
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
        }
    }
};

YoloxTensorrt::YoloxTensorrt(
    const std::string& engine_path,
    const double       score_threshold,
    const double       nms_threshold)
    : implementation_(std::make_unique<Implementation>()) {
    implementation_->score_threshold = score_threshold;
    implementation_->nms_threshold   = nms_threshold;

    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("TensorRTエンジンを開けません: " + engine_path);
    }
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    implementation_->runtime.reset(nvinfer1::createInferRuntime(implementation_->logger));
    implementation_->engine.reset(implementation_->runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!implementation_->engine) {
        throw std::runtime_error(
            "TensorRTエンジンの復元に失敗しました(実機・TensorRTバージョンごとに再生成が必要): " +
            engine_path + " " + implementation_->logger.last_message());
    }
    implementation_->context.reset(implementation_->engine->createExecutionContext());

    for (int index = 0; index < implementation_->engine->getNbIOTensors(); ++index) {
        const char* const tensor_name = implementation_->engine->getIOTensorName(index);
        if (implementation_->engine->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT) {
            implementation_->input_name = tensor_name;
        } else {
            implementation_->output_name = tensor_name;
        }
    }

    // 入力 [1,3,H,W] / 出力 [1,anchors,5+classes]
    const nvinfer1::Dims input_dims  = implementation_->engine->getTensorShape(implementation_->input_name.c_str());
    const nvinfer1::Dims output_dims = implementation_->engine->getTensorShape(implementation_->output_name.c_str());
    implementation_->input_height    = input_dims.d[2];
    implementation_->input_width     = input_dims.d[3];
    implementation_->num_anchors     = output_dims.d[1];
    implementation_->num_attributes  = output_dims.d[2];

    int expected_anchors = 0;
    for (const int stride : strides) {
        expected_anchors += (implementation_->input_height / stride) * (implementation_->input_width / stride);
    }
    if (expected_anchors <= 0 || implementation_->num_anchors != expected_anchors || implementation_->num_attributes < 5) {
        throw std::runtime_error(
            "TensorRTエンジンの形状がYOLOXの想定と一致しません: input=" +
            std::to_string(implementation_->input_height) + "x" +
            std::to_string(implementation_->input_width) +
            ", anchors=" + std::to_string(implementation_->num_anchors) + "(expected " +
            std::to_string(expected_anchors) +
            "), attributes=" + std::to_string(implementation_->num_attributes));
    }

    implementation_->input_host.resize(static_cast<size_t>(3) * implementation_->input_height * implementation_->input_width);
    implementation_->output_host.resize(static_cast<size_t>(implementation_->num_anchors) * implementation_->num_attributes);

    cudaMalloc(&implementation_->input_device, implementation_->input_host.size() * sizeof(float));
    cudaMalloc(&implementation_->output_device, implementation_->output_host.size() * sizeof(float));
    cudaStreamCreate(&implementation_->stream);
}

YoloxTensorrt::~YoloxTensorrt() = default;

std::vector<Detection> YoloxTensorrt::detect(const cv::Mat& bgr_image) {
    cv::Mat      padded;
    const double ratio = letterbox(bgr_image, implementation_->input_width, implementation_->input_height, padded);
    to_nchw(padded, implementation_->input_host);

    const size_t input_bytes  = implementation_->input_host.size() * sizeof(float);
    const size_t output_bytes = implementation_->output_host.size() * sizeof(float);
    cudaMemcpyAsync(implementation_->input_device, implementation_->input_host.data(), input_bytes, cudaMemcpyHostToDevice, implementation_->stream);
    implementation_->context->setTensorAddress(implementation_->input_name.c_str(), implementation_->input_device);
    implementation_->context->setTensorAddress(implementation_->output_name.c_str(), implementation_->output_device);
    implementation_->context->enqueueV3(implementation_->stream);
    cudaMemcpyAsync(implementation_->output_host.data(), implementation_->output_device, output_bytes, cudaMemcpyDeviceToHost, implementation_->stream);
    cudaStreamSynchronize(implementation_->stream);

    std::vector<cv::Rect2d> boxes;
    std::vector<float>      scores;
    std::vector<int>        class_ids;
    decode(implementation_->output_host.data(), implementation_->num_attributes, implementation_->input_width, implementation_->input_height, implementation_->score_threshold, boxes, scores, class_ids);

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(implementation_->score_threshold), static_cast<float>(implementation_->nms_threshold), keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (const int index : keep) {
        // レターボックス前の画素座標へ戻す
        const cv::Rect2d& box = boxes[index];
        detections.push_back(Detection{
            cv::Rect2d(box.x / ratio, box.y / ratio, box.width / ratio, box.height / ratio),
            class_ids[index],
            scores[index]});
    }
    return detections;
}

}  // namespace object_detection
