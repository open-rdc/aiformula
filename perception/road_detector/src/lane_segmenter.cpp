#include "road_detector/lane_segmenter.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "road_detector/letterbox.hpp"

namespace road_detector {

namespace {

class TensorrtLogger : public nvinfer1::ILogger {
    void log(const Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

// 成功なら空文字列、失敗なら理由を返す
std::string cuda_error(const cudaError_t status, const char* const what)
{
    return status == cudaSuccess ? std::string() : std::string("CUDA エラー(") + what + "): " + cudaGetErrorString(status);
}

}  // namespace

class LaneSegmenter::Implementation {
   public:
    TensorrtLogger                               logger;
    std::unique_ptr<nvinfer1::IRuntime>          runtime;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t                                 stream = nullptr;

    std::string input_name;
    std::string output_name;
    int         input_width  = 0;
    int         input_height = 0;

    void*              input_device  = nullptr;
    void*              output_device = nullptr;
    std::vector<float> input_host;
    cv::Mat            output_host;
    std::string        last_error;

    ~Implementation()
    {
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

LaneSegmenter::LaneSegmenter(const std::string& engine_path)
    : implementation_(std::make_unique<Implementation>())
{
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("TensorRTエンジンを開けません: " + engine_path);
    }
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    implementation_->runtime.reset(nvinfer1::createInferRuntime(implementation_->logger));
    implementation_->engine.reset(implementation_->runtime->deserializeCudaEngine(blob.data(), blob.size()));
    implementation_->context.reset(implementation_->engine->createExecutionContext());

    for (int index = 0; index < implementation_->engine->getNbIOTensors(); ++index) {
        const char* const tensor_name = implementation_->engine->getIOTensorName(index);
        if (implementation_->engine->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT) {
            implementation_->input_name = tensor_name;
        } else {
            implementation_->output_name = tensor_name;
        }
    }

    // 入力 [1,3,H,W] / 出力 [1,1,H,W]
    const nvinfer1::Dims input_dims = implementation_->engine->getTensorShape(implementation_->input_name.c_str());
    implementation_->input_height   = input_dims.d[2];
    implementation_->input_width    = input_dims.d[3];

    implementation_->input_host.resize(static_cast<size_t>(3) * implementation_->input_height * implementation_->input_width);
    implementation_->output_host = cv::Mat(implementation_->input_height, implementation_->input_width, CV_32FC1);

    cudaMalloc(&implementation_->input_device, implementation_->input_host.size() * sizeof(float));
    cudaMalloc(&implementation_->output_device, implementation_->output_host.total() * sizeof(float));
    cudaStreamCreate(&implementation_->stream);
}

LaneSegmenter::~LaneSegmenter() = default;

cv::Size LaneSegmenter::input_size() const { return cv::Size(implementation_->input_width, implementation_->input_height); }

const std::string& LaneSegmenter::last_error() const { return implementation_->last_error; }

cv::Mat LaneSegmenter::infer(const cv::Mat& padded_bgr)
{
    implementation_->last_error.clear();

    // 抜ける前に stream を回収する。放置すると次フレームの input_host 書き換えが in-flight の H2D と競合する
    const auto degrade = [this](const std::string& reason) {
        cudaStreamSynchronize(implementation_->stream);
        implementation_->last_error = reason;
        return cv::Mat();
    };

    if (padded_bgr.type() != CV_8UC3 || padded_bgr.size() != input_size()) {
        return degrade("入力画像の型かサイズが engine と一致しません");
    }

    to_nchw(padded_bgr, implementation_->input_host);

    const size_t input_bytes  = implementation_->input_host.size() * sizeof(float);
    const size_t output_bytes = implementation_->output_host.total() * sizeof(float);

    if (const std::string error = cuda_error(cudaMemcpyAsync(implementation_->input_device, implementation_->input_host.data(), input_bytes, cudaMemcpyHostToDevice, implementation_->stream), "cudaMemcpyAsync(H2D)"); !error.empty()) {
        return degrade(error);
    }
    if (!implementation_->context->setTensorAddress(implementation_->input_name.c_str(), implementation_->input_device) ||
        !implementation_->context->setTensorAddress(implementation_->output_name.c_str(), implementation_->output_device)) {
        return degrade("TensorRTのテンソルアドレス設定に失敗しました");
    }
    if (!implementation_->context->enqueueV3(implementation_->stream)) {
        return degrade("TensorRT推論に失敗しました");
    }
    if (const std::string error = cuda_error(cudaMemcpyAsync(implementation_->output_host.data, implementation_->output_device, output_bytes, cudaMemcpyDeviceToHost, implementation_->stream), "cudaMemcpyAsync(D2H)"); !error.empty()) {
        return degrade(error);
    }
    // 非同期カーネルの実行時エラーはここで初めて表面化する
    if (const std::string error = cuda_error(cudaStreamSynchronize(implementation_->stream), "cudaStreamSynchronize"); !error.empty()) {
        return degrade(error);
    }

    return implementation_->output_host;
}

}  // namespace road_detector
