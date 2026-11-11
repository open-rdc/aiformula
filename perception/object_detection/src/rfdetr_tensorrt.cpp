#include "object_detection/rfdetr_tensorrt.hpp"

#include <opencv2/imgproc.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace object_detection {

namespace {

// RF-DETR の学習時の正規化(ImageNet の RGB 平均と標準偏差)
constexpr float mean[] = {0.485F, 0.456F, 0.406F};
constexpr float stddev[] = {0.229F, 0.224F, 0.225F};

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

}  // namespace

class RfdetrTensorrt::Implementation {
   public:
    TensorrtLogger                               logger;
    std::unique_ptr<nvinfer1::IRuntime>          runtime;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t                                 stream = nullptr;

    int input_width  = 0;
    int input_height = 0;
    int num_queries  = 0;
    int num_classes  = 0;

    void*              input_device  = nullptr;
    void*              boxes_device  = nullptr;
    void*              logits_device = nullptr;
    std::vector<float> input_host;
    std::vector<float> boxes_host;
    std::vector<float> logits_host;

    double score_threshold = 0.0;

    ~Implementation() {
        cudaFree(input_device);
        cudaFree(boxes_device);
        cudaFree(logits_device);
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
        }
    }
};

RfdetrTensorrt::RfdetrTensorrt(const std::string& engine_path, const double score_threshold)
    : implementation_(std::make_unique<Implementation>()) {
    implementation_->score_threshold = score_threshold;

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

    // 入力 input [1,3,H,W] / 出力 dets [1,queries,4](正規化 cxcywh)・labels [1,queries,classes+1](ロジット)
    const nvinfer1::Dims input_dims  = implementation_->engine->getTensorShape("input");
    const nvinfer1::Dims logits_dims = implementation_->engine->getTensorShape("labels");
    implementation_->input_height    = input_dims.d[2];
    implementation_->input_width     = input_dims.d[3];
    implementation_->num_queries     = logits_dims.d[1];
    // 最後の 1 つは学習データに無いクラス(COCO のカテゴリ ID が 1 始まりのため)
    implementation_->num_classes     = logits_dims.d[2] - 1;

    implementation_->input_host.resize(static_cast<size_t>(3) * implementation_->input_height * implementation_->input_width);
    implementation_->boxes_host.resize(static_cast<size_t>(implementation_->num_queries) * 4);
    implementation_->logits_host.resize(static_cast<size_t>(implementation_->num_queries) * logits_dims.d[2]);

    cudaMalloc(&implementation_->input_device, implementation_->input_host.size() * sizeof(float));
    cudaMalloc(&implementation_->boxes_device, implementation_->boxes_host.size() * sizeof(float));
    cudaMalloc(&implementation_->logits_device, implementation_->logits_host.size() * sizeof(float));
    cudaStreamCreate(&implementation_->stream);
    implementation_->context->setTensorAddress("input", implementation_->input_device);
    implementation_->context->setTensorAddress("dets", implementation_->boxes_device);
    implementation_->context->setTensorAddress("labels", implementation_->logits_device);
}

RfdetrTensorrt::~RfdetrTensorrt() = default;

std::vector<Detection> RfdetrTensorrt::detect(const cv::Mat& bgr_image) {
    // 1. RGB に変えて入力サイズへ引き伸ばし、正規化して NCHW に並べる
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(implementation_->input_width, implementation_->input_height), 0.0, 0.0, cv::INTER_LINEAR);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    const size_t plane = static_cast<size_t>(implementation_->input_width) * implementation_->input_height;
    for (int y = 0; y < resized.rows; ++y) {
        const uint8_t* row = resized.ptr<uint8_t>(y);
        for (int x = 0; x < resized.cols; ++x) {
            for (int channel = 0; channel < 3; ++channel) {
                implementation_->input_host[channel * plane + static_cast<size_t>(y) * resized.cols + x] =
                    (row[x * 3 + channel] / 255.0F - mean[channel]) / stddev[channel];
            }
        }
    }

    // 2. 推論する
    cudaMemcpyAsync(implementation_->input_device, implementation_->input_host.data(), implementation_->input_host.size() * sizeof(float), cudaMemcpyHostToDevice, implementation_->stream);
    implementation_->context->enqueueV3(implementation_->stream);
    cudaMemcpyAsync(implementation_->boxes_host.data(), implementation_->boxes_device, implementation_->boxes_host.size() * sizeof(float), cudaMemcpyDeviceToHost, implementation_->stream);
    cudaMemcpyAsync(implementation_->logits_host.data(), implementation_->logits_device, implementation_->logits_host.size() * sizeof(float), cudaMemcpyDeviceToHost, implementation_->stream);
    cudaStreamSynchronize(implementation_->stream);

    // 3. クエリごとに最大のクラスを取り、スコアが閾値以上のものを元画像の画素座標へ戻す(DETR は重複を出さないので NMS は不要)
    std::vector<Detection> detections;
    const int stride = implementation_->num_classes + 1;
    for (int query = 0; query < implementation_->num_queries; ++query) {
        const float* logits   = implementation_->logits_host.data() + static_cast<size_t>(query) * stride;
        int          class_id = 0;
        for (int c = 1; c < implementation_->num_classes; ++c) {
            if (logits[c] > logits[class_id]) {
                class_id = c;
            }
        }
        const float score = 1.0F / (1.0F + std::exp(-logits[class_id]));
        if (score < implementation_->score_threshold) {
            continue;
        }
        const float* box = implementation_->boxes_host.data() + static_cast<size_t>(query) * 4;
        detections.push_back(Detection{
            cv::Rect2d((box[0] - box[2] * 0.5) * bgr_image.cols, (box[1] - box[3] * 0.5) * bgr_image.rows, box[2] * bgr_image.cols, box[3] * bgr_image.rows),
            class_id,
            score});
    }
    return detections;
}

}  // namespace object_detection
