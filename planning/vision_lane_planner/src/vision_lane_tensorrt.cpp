#include "vision_lane_planner/vision_lane_tensorrt.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace vision_lane_planner {

namespace {

constexpr const char* input_name    = "input";
constexpr const char* exist_name    = "exist";
constexpr const char* valid_name    = "valid";
constexpr const char* position_name = "position";

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

void to_nchw(const cv::Mat& padded, std::vector<float>& blob) {
    const int width  = padded.cols;
    const int height = padded.rows;
    blob.resize(static_cast<std::size_t>(3) * width * height);
    for (int channel = 0; channel < 3; ++channel) {
        for (int y = 0; y < height; ++y) {
            const uint8_t* row = padded.ptr<uint8_t>(y);
            float*         out = blob.data() + (static_cast<std::size_t>(channel) * height + y) * width;
            for (int x = 0; x < width; ++x) {
                out[x] = static_cast<float>(row[x * 3 + channel]) / 255.0F;
            }
        }
    }
}

}  // namespace

class VisionLaneTensorrt::Implementation {
   public:
    TensorrtLogger                               logger;
    std::unique_ptr<nvinfer1::IRuntime>          runtime;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;

    std::vector<float>                      input_host;
    std::array<float, num_slots>            exist_host{};
    std::array<float, num_slots * num_rows> valid_host{};
    std::array<float, num_slots * num_rows> position_host{};

    void*        input_device    = nullptr;
    void*        exist_device    = nullptr;
    void*        valid_device    = nullptr;
    void*        position_device = nullptr;
    cudaStream_t stream          = nullptr;

    std::string last_error;

    ~Implementation() {
        for (void* buffer : {input_device, exist_device, valid_device, position_device}) {
            if (buffer != nullptr) {
                cudaFree(buffer);
            }
        }
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
        }
    }
};

VisionLaneTensorrt::VisionLaneTensorrt(const std::string& engine_path) : implementation_(std::make_unique<Implementation>()) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("TensorRTエンジンを開けません: " + engine_path);
    }
    const std::vector<char> blob(
        (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    implementation_->runtime.reset(nvinfer1::createInferRuntime(implementation_->logger));
    implementation_->engine.reset(
        implementation_->runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!implementation_->engine) {
        throw std::runtime_error(
            "TensorRTエンジンの復元に失敗しました(実機・TensorRTバージョンごとに再生成が必要): " +
            engine_path + " " + implementation_->logger.last_message());
    }
    implementation_->context.reset(implementation_->engine->createExecutionContext());

    const nvinfer1::Dims input_dims = implementation_->engine->getTensorShape(input_name);
    if (input_dims.nbDims != 4 || input_dims.d[2] != input_height || input_dims.d[3] != input_width) {
        throw std::runtime_error(
            "TensorRTエンジンの入力が " + std::to_string(input_height) + "x" +
            std::to_string(input_width) + " ではありません: " + engine_path);
    }

    implementation_->input_host.resize(static_cast<std::size_t>(3) * input_height * input_width);

    cudaMalloc(&implementation_->input_device, implementation_->input_host.size() * sizeof(float));
    cudaMalloc(&implementation_->exist_device, implementation_->exist_host.size() * sizeof(float));
    cudaMalloc(&implementation_->valid_device, implementation_->valid_host.size() * sizeof(float));
    cudaMalloc(&implementation_->position_device, implementation_->position_host.size() * sizeof(float));
    cudaStreamCreate(&implementation_->stream);
}

VisionLaneTensorrt::~VisionLaneTensorrt() = default;

const std::string& VisionLaneTensorrt::last_error() const {
    return implementation_->last_error;
}

const std::string& VisionLaneTensorrt::last_warning() const {
    return implementation_->logger.last_message();
}

SlotPrediction VisionLaneTensorrt::infer(const cv::Mat& padded) {
    implementation_->last_error.clear();
    SlotPrediction prediction;

    const auto degrade = [this, &prediction](const std::string& reason) {
        cudaStreamSynchronize(implementation_->stream);
        implementation_->last_error = reason;
        return prediction;
    };

    if (padded.type() != CV_8UC3 || padded.cols != input_width || padded.rows != input_height) {
        return degrade("入力画像の型かサイズがengineと一致しません");
    }

    to_nchw(padded, implementation_->input_host);
    cudaMemcpyAsync(
        implementation_->input_device, implementation_->input_host.data(),
        implementation_->input_host.size() * sizeof(float), cudaMemcpyHostToDevice,
        implementation_->stream);

    if (!implementation_->context->setTensorAddress(input_name, implementation_->input_device) ||
        !implementation_->context->setTensorAddress(exist_name, implementation_->exist_device) ||
        !implementation_->context->setTensorAddress(valid_name, implementation_->valid_device) ||
        !implementation_->context->setTensorAddress(position_name, implementation_->position_device)) {
        return degrade("TensorRTのテンソルアドレス設定に失敗しました");
    }
    if (!implementation_->context->enqueueV3(implementation_->stream)) {
        return degrade("TensorRT推論に失敗しました");
    }

    cudaMemcpyAsync(
        implementation_->exist_host.data(), implementation_->exist_device,
        implementation_->exist_host.size() * sizeof(float), cudaMemcpyDeviceToHost,
        implementation_->stream);
    cudaMemcpyAsync(
        implementation_->valid_host.data(), implementation_->valid_device,
        implementation_->valid_host.size() * sizeof(float), cudaMemcpyDeviceToHost,
        implementation_->stream);
    cudaMemcpyAsync(
        implementation_->position_host.data(), implementation_->position_device,
        implementation_->position_host.size() * sizeof(float), cudaMemcpyDeviceToHost,
        implementation_->stream);

    if (const cudaError_t status = cudaStreamSynchronize(implementation_->stream);
        status != cudaSuccess) {
        return degrade(cudaGetErrorString(status));
    }

    for (std::size_t slot = 0; slot < num_slots; ++slot) {
        prediction.exist[slot] = implementation_->exist_host[slot];
        for (std::size_t row = 0; row < num_rows; ++row) {
            const std::size_t index           = slot * num_rows + row;
            prediction.valid_logit[slot][row] = implementation_->valid_host[index];
            prediction.position[slot][row]    = implementation_->position_host[index];
        }
    }
    prediction.valid = true;
    return prediction;
}

}  // namespace vision_lane_planner
