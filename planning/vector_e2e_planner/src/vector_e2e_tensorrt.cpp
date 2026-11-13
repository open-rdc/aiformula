#include "vector_e2e_planner/vector_e2e_tensorrt.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <vector>

namespace vector_e2e_planner {

namespace {

constexpr const char* input_name = "image";

// 経路選択・信号停止に使う出力。残りの出力（地図・物体検出）は読まない
constexpr const char* branch_exist_name  = "branch_exist";
constexpr const char* path_xy_name       = "path_xy";
constexpr const char* path_valid_name    = "path_valid";
constexpr const char* corridor_left_name = "corridor_left";
constexpr const char* corridor_right_name = "corridor_right";
constexpr const char* signal_prob_name   = "signal_prob";
constexpr const char* signal_xy_name     = "signal_xy";

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

std::size_t element_size(const nvinfer1::DataType type) {
    switch (type) {
        case nvinfer1::DataType::kFLOAT:
        case nvinfer1::DataType::kINT32:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kINT64:
            return 8;
        default:
            return 1;
    }
}

// BGR のまま CHW・0..1 へ。学習は cv2.imread の BGR をそのまま入力している
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

class VectorE2eTensorrt::Implementation {
   public:
    struct Buffer {
        void*             device = nullptr;
        std::vector<char> host;
        std::size_t       count = 0;  // 要素数
        bool              is_float = false;
    };

    TensorrtLogger                               logger;
    std::unique_ptr<nvinfer1::IRuntime>          runtime;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;

    std::vector<float>            input_host;
    void*                         input_device = nullptr;
    std::map<std::string, Buffer> outputs;  // 出力テンソルすべて（TensorRT は全出力のアドレス設定を要求する）
    cudaStream_t                  stream = nullptr;

    std::string last_error;

    ~Implementation() {
        if (input_device != nullptr) {
            cudaFree(input_device);
        }
        for (auto& entry : outputs) {
            if (entry.second.device != nullptr) {
                cudaFree(entry.second.device);
            }
        }
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
        }
    }

    const float* output(const char* name) const {
        return reinterpret_cast<const float*>(outputs.at(name).host.data());
    }
};

VectorE2eTensorrt::VectorE2eTensorrt(const std::string& engine_path) : implementation_(std::make_unique<Implementation>()) {
    Implementation& impl = *implementation_;

    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("TensorRTエンジンを開けません: " + engine_path);
    }
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    impl.runtime.reset(nvinfer1::createInferRuntime(impl.logger));
    impl.engine.reset(impl.runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!impl.engine) {
        throw std::runtime_error(
            "TensorRTエンジンの復元に失敗しました(実機・TensorRTバージョンごとに再生成が必要): " + engine_path + " " +
            impl.logger.last_message());
    }
    impl.context.reset(impl.engine->createExecutionContext());

    const nvinfer1::Dims input_dims = impl.engine->getTensorShape(input_name);
    if (input_dims.nbDims != 4 || input_dims.d[2] != input_height || input_dims.d[3] != input_width) {
        throw std::runtime_error(
            "TensorRTエンジンの入力が " + std::to_string(input_height) + "x" + std::to_string(input_width) +
            " ではありません: " + engine_path);
    }
    impl.input_host.resize(static_cast<std::size_t>(3) * input_height * input_width);
    cudaMalloc(&impl.input_device, impl.input_host.size() * sizeof(float));

    // 出力は名前で引く。dtype・形はエンジンから取る（agent_cls は int64 から int32 へ変わる等）
    for (int i = 0; i < impl.engine->getNbIOTensors(); ++i) {
        const char* name = impl.engine->getIOTensorName(i);
        if (impl.engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT) {
            continue;
        }
        const nvinfer1::Dims dims  = impl.engine->getTensorShape(name);
        std::size_t          count = 1;
        for (int d = 0; d < dims.nbDims; ++d) {
            count *= static_cast<std::size_t>(dims.d[d]);
        }
        const nvinfer1::DataType type = impl.engine->getTensorDataType(name);
        Implementation::Buffer   buffer;
        buffer.count    = count;
        buffer.is_float = type == nvinfer1::DataType::kFLOAT;
        buffer.host.resize(count * element_size(type));
        cudaMalloc(&buffer.device, buffer.host.size());
        impl.outputs.emplace(name, std::move(buffer));
    }

    const auto require = [&](const char* name, const std::size_t count) {
        const auto found = impl.outputs.find(name);
        if (found == impl.outputs.end() || !found->second.is_float || found->second.count != count) {
            throw std::runtime_error(std::string("TensorRTエンジンの出力が契約と一致しません: ") + name);
        }
    };
    require(branch_exist_name, num_slots);
    require(path_xy_name, num_slots * num_path_points * 2);
    require(path_valid_name, num_slots * num_path_points);
    require(corridor_left_name, num_slots * num_path_points * 2);
    require(corridor_right_name, num_slots * num_path_points * 2);
    require(signal_prob_name, num_signal_class);
    require(signal_xy_name, 2);
    cudaStreamCreate(&impl.stream);
}

VectorE2eTensorrt::~VectorE2eTensorrt() = default;

const std::string& VectorE2eTensorrt::last_error() const {
    return implementation_->last_error;
}

ModelOutputs VectorE2eTensorrt::infer(const cv::Mat& padded) {
    Implementation& impl = *implementation_;
    impl.last_error.clear();
    ModelOutputs result;

    const auto degrade = [&impl, &result](const std::string& reason) {
        cudaStreamSynchronize(impl.stream);
        impl.last_error = reason;
        return result;
    };

    if (padded.type() != CV_8UC3 || padded.cols != input_width || padded.rows != input_height) {
        return degrade("入力画像の型かサイズがengineと一致しません");
    }

    to_nchw(padded, impl.input_host);
    cudaMemcpyAsync(
        impl.input_device, impl.input_host.data(), impl.input_host.size() * sizeof(float), cudaMemcpyHostToDevice,
        impl.stream);

    bool addresses_ok = impl.context->setTensorAddress(input_name, impl.input_device);
    for (auto& entry : impl.outputs) {
        addresses_ok = impl.context->setTensorAddress(entry.first.c_str(), entry.second.device) && addresses_ok;
    }
    if (!addresses_ok) {
        return degrade("TensorRTのテンソルアドレス設定に失敗しました");
    }
    if (!impl.context->enqueueV3(impl.stream)) {
        return degrade("TensorRT推論に失敗しました");
    }

    for (const char* name : {branch_exist_name, path_xy_name, path_valid_name, corridor_left_name, corridor_right_name,
                             signal_prob_name, signal_xy_name}) {
        Implementation::Buffer& buffer = impl.outputs.at(name);
        cudaMemcpyAsync(buffer.host.data(), buffer.device, buffer.host.size(), cudaMemcpyDeviceToHost, impl.stream);
    }
    if (const cudaError_t status = cudaStreamSynchronize(impl.stream); status != cudaSuccess) {
        return degrade(cudaGetErrorString(status));
    }

    const float* exist = impl.output(branch_exist_name);
    const float* path  = impl.output(path_xy_name);
    const float* valid = impl.output(path_valid_name);
    const float* left  = impl.output(corridor_left_name);
    const float* right = impl.output(corridor_right_name);
    for (std::size_t slot = 0; slot < num_slots; ++slot) {
        result.branch_exist[slot] = static_cast<double>(exist[slot]);
        for (std::size_t i = 0; i < num_path_points; ++i) {
            const std::size_t index = slot * num_path_points + i;
            result.path_valid[slot][i]     = static_cast<double>(valid[index]);
            result.path_xy[slot][i]        = Point2D{path[index * 2], path[index * 2 + 1]};
            result.corridor_left[slot][i]  = Point2D{left[index * 2], left[index * 2 + 1]};
            result.corridor_right[slot][i] = Point2D{right[index * 2], right[index * 2 + 1]};
        }
    }
    const float* prob = impl.output(signal_prob_name);
    const float* xy   = impl.output(signal_xy_name);
    for (std::size_t i = 0; i < num_signal_class; ++i) {
        result.signal_prob[i] = static_cast<double>(prob[i]);
    }
    result.signal_xy = Point2D{xy[0], xy[1]};
    result.valid = true;
    return result;
}

}  // namespace vector_e2e_planner
