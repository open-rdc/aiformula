#include "utilities/velplanner.hpp"
#include "utilities/utils.hpp"
#include <algorithm>

using namespace utils;

namespace velplanner{

rclcpp::Clock system_clock(RCL_ROS_TIME);

VelPlanner::VelPlanner() 
    : rng_(std::random_device{}()), noise_dist_(0.0, 1.0), device_(torch::kCPU), gpu_available_(false) {
    initializeDevice();
    initializeTensors();
}

bool VelPlanner::loadModel(const std::string& model_path) {
    model_ = torch::jit::load(model_path, device_);
    model_.eval();
    if (gpu_available_) {
        model_.to(device_);
    }
    model_loaded_ = true;
    RCLCPP_INFO(rclcpp::get_logger("velplanner"), "MPPI model loaded successfully (%s): %s", 
               gpu_available_ ? "GPU" : "CPU", model_path.c_str());
    return true;
}

// GPU/CPUデバイス初期化
void VelPlanner::initializeDevice() {
    gpu_available_ = torch::cuda::is_available();
    if (gpu_available_) {
        device_ = torch::Device(torch::kCUDA, 0);
        RCLCPP_INFO(rclcpp::get_logger("velplanner"), "GPU available, using CUDA device 0");
    } else {
        device_ = torch::Device(torch::kCPU);
        RCLCPP_INFO(rclcpp::get_logger("velplanner"), "GPU not available, using CPU");
    }
}

void VelPlanner::initializeTensors() {
    const auto tensor_options = torch::TensorOptions().dtype(torch::kFloat32).device(device_);
    
    samples_tensor_ = torch::zeros({mppi_params_.num_samples, mppi_params_.horizon, 2}, tensor_options);
    states_tensor_ = torch::zeros({mppi_params_.num_samples, mppi_params_.horizon + 1, 7}, tensor_options);
    costs_tensor_ = torch::zeros({mppi_params_.num_samples}, tensor_options);
    
    RCLCPP_INFO(rclcpp::get_logger("velplanner"), "MPPI tensors initialized on %s", 
               gpu_available_ ? "GPU" : "CPU");
}


void VelPlanner::cycle(const State_t state) {
    current_state_ = state;
    
    std::array<float, 7> input = {
        static_cast<float>(state.linear_vel),
        static_cast<float>(state.angular_vel),
        static_cast<float>(state.acc_linear_vel),
        static_cast<float>(state.acc_angular_vel),
        static_cast<float>(state.potentio),
        static_cast<float>(optimal_linear_),
        static_cast<float>(optimal_angular_)
    };
    
    state_buffer_.push_back(input);
    if (state_buffer_.size() > static_cast<size_t>(seq_len_)) {
        state_buffer_.pop_front();
    }
    
    if (model_loaded_ && state_buffer_.size() == static_cast<size_t>(seq_len_)) {
        torch::Tensor control_samples = generateControlSamples();
        
        torch::Tensor predicted_states = rolloutModel(control_samples);
        
        torch::Tensor costs = computeCosts(predicted_states, control_samples);
        
        auto optimal_control = computeOptimalControl(control_samples, costs);
        optimal_linear_ = optimal_control.first;
        optimal_angular_ = optimal_control.second;
    } else if (!model_loaded_) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("velplanner"), system_clock, 5000,
                             "MPPI model not loaded. Call loadModel() first.");
    }
}

torch::Tensor VelPlanner::generateControlSamples() const {
    samples_tensor_.normal_(0.0, 1.0);
    
    samples_tensor_.select(2, 0).mul_(mppi_params_.noise_sigma_linear);
    samples_tensor_.select(2, 1).mul_(mppi_params_.noise_sigma_angular);
    
    samples_tensor_.select(2, 0).clamp_(-1.0, 1.0);
    samples_tensor_.select(2, 1).clamp_(-2.0, 2.0);
    
    return samples_tensor_;
}

torch::Tensor VelPlanner::rolloutModel(const torch::Tensor& control_sequences) const {
    const int num_samples = control_sequences.size(0);
    const int horizon = control_sequences.size(1);
    
    torch::Device device = control_sequences.device();
    
    torch::Tensor predicted_states = states_tensor_.narrow(0, 0, num_samples).narrow(1, 0, horizon + 1);
    predicted_states.zero_();
    
    predicted_states.select(1, 0).select(1, 0).fill_(current_state_.linear_vel);
    predicted_states.select(1, 0).select(1, 1).fill_(current_state_.angular_vel);
    predicted_states.select(1, 0).select(1, 2).fill_(current_state_.acc_linear_vel);
    predicted_states.select(1, 0).select(1, 3).fill_(current_state_.acc_angular_vel);
    predicted_states.select(1, 0).select(1, 4).fill_(current_state_.potentio);
    
    torch::Tensor base_state_buffer = torch::zeros({seq_len_, 7}, 
                                                    torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    const int buffer_size = static_cast<int>(state_buffer_.size());
    const int copy_size = std::min(seq_len_, buffer_size);
    
    auto buffer_accessor = base_state_buffer.accessor<float, 2>();
    for (int j = 0; j < copy_size; ++j) {
        const auto& state = state_buffer_[j];
        buffer_accessor[j][0] = state[0];
        buffer_accessor[j][1] = state[1];
        buffer_accessor[j][2] = state[2];
        buffer_accessor[j][3] = state[3];
        buffer_accessor[j][4] = state[4];
        buffer_accessor[j][5] = state[5];
        buffer_accessor[j][6] = state[6];
    }
    
    // バッチ入力テンソルの準備（全サンプル共通部分）
    torch::Tensor batch_input = base_state_buffer.unsqueeze(0).expand({num_samples, seq_len_, 7}).clone();
    
    // 時間方向のロールアウト（並列処理対応）
    torch::NoGradGuard no_grad;
    
    for (int t = 0; t < horizon; ++t) {
        // 制御入力を最新ステップに設定
        batch_input.select(1, seq_len_ - 1).select(1, 5) = control_sequences.select(1, t).select(1, 0);
        batch_input.select(1, seq_len_ - 1).select(1, 6) = control_sequences.select(1, t).select(1, 1);
        
        // バッチ推論（JIT最適化）
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(batch_input);
        
        torch::Tensor output = model_.forward(inputs).toTensor();
        
        // 次の状態を更新（インプレース操作で高速化）
        predicted_states.select(1, t + 1).select(1, 0).copy_(output.select(1, 0));
        predicted_states.select(1, t + 1).select(1, 1).copy_(output.select(1, 1));
        predicted_states.select(1, t + 1).select(1, 4).copy_(output.select(1, 2));
        
        // 加速度の数値微分計算（ベクトル化）
        if (t > 0) {
            predicted_states.select(1, t + 1).select(1, 2).copy_(
                (predicted_states.select(1, t + 1).select(1, 0) - 
                 predicted_states.select(1, t).select(1, 0)) / mppi_params_.dt);
            predicted_states.select(1, t + 1).select(1, 3).copy_(
                (predicted_states.select(1, t + 1).select(1, 1) - 
                 predicted_states.select(1, t).select(1, 1)) / mppi_params_.dt);
        }
        
        // 制御入力の設定
        predicted_states.select(1, t + 1).select(1, 5).copy_(control_sequences.select(1, t).select(1, 0));
        predicted_states.select(1, t + 1).select(1, 6).copy_(control_sequences.select(1, t).select(1, 1));
        
        if (t < horizon - 1) {
            batch_input.narrow(1, 0, seq_len_ - 1).copy_(batch_input.narrow(1, 1, seq_len_ - 1));
            // 最新の状態を末尾に追加
            batch_input.select(1, seq_len_ - 1).select(1, 0).copy_(predicted_states.select(1, t + 1).select(1, 0));
            batch_input.select(1, seq_len_ - 1).select(1, 1).copy_(predicted_states.select(1, t + 1).select(1, 1));
            batch_input.select(1, seq_len_ - 1).select(1, 2).copy_(predicted_states.select(1, t + 1).select(1, 2));
            batch_input.select(1, seq_len_ - 1).select(1, 3).copy_(predicted_states.select(1, t + 1).select(1, 3));
            batch_input.select(1, seq_len_ - 1).select(1, 4).copy_(predicted_states.select(1, t + 1).select(1, 4));
        }
    }
    
    return predicted_states;
}

// コスト計算（高速化：インプレース演算、重み事前計算）
torch::Tensor VelPlanner::computeCosts(const torch::Tensor& predicted_states, 
                                      const torch::Tensor& control_sequences) const {
    const int num_samples = predicted_states.size(0);
    const int horizon = predicted_states.size(1) - 1;
    
    torch::Tensor costs = costs_tensor_.narrow(0, 0, num_samples);
    costs.zero_();
    
    const auto vel_states = predicted_states.narrow(1, 1, horizon);  // t=1からhorizonまで
    auto linear_error = vel_states.select(2, 0) - target_linear_;
    linear_error.square_();  // インプレース
    auto angular_error = vel_states.select(2, 1) - target_angular_;
    angular_error.square_(); // インプレース
    costs.add_(torch::sum(linear_error + angular_error, 1), mppi_params_.weight_vel_error);
    
    auto control_cost = torch::sum(torch::sum(control_sequences.square(), 2), 1);
    costs.add_(control_cost, mppi_params_.weight_control_effort);
    
    if (horizon > 1) [[likely]] {
        auto control_diff = control_sequences.narrow(1, 1, horizon - 1) - 
                           control_sequences.narrow(1, 0, horizon - 1);
        auto smoothness_cost = torch::sum(torch::sum(control_diff.square(), 2), 1);
        costs.add_(smoothness_cost, mppi_params_.weight_smoothness);
    }
    
    return costs;
}

// 最適制御の計算（高速化：インプレース演算、直接計算）
std::pair<double, double> VelPlanner::computeOptimalControl(const torch::Tensor& control_samples,
                                                           const torch::Tensor& costs) const {
    const float inv_lambda = 1.0f / mppi_params_.lambda;
    auto weights = torch::exp(costs * (-inv_lambda));  // 乗算の方が除算より高速
    weights.div_(torch::sum(weights));  // インプレース正規化
    
    // 最初のステップの制御入力のみ計算（receding horizon、コピー削除）
    const auto first_step_controls = control_samples.select(1, 0);  // [num_samples, 2]
    
    // 重み付き平均の計算（ベクトル化、一発計算）
    const auto optimal_controls = torch::sum(weights.unsqueeze(1) * first_step_controls, 0);
    
    const auto cpu_controls = optimal_controls.device() != torch::kCPU ? 
                             optimal_controls.to(torch::kCPU) : optimal_controls;
    
    constexpr double linear_min = -1.0, linear_max = 1.0;
    constexpr double angular_min = -2.0, angular_max = 2.0;
    
    return {
        std::clamp(cpu_controls[0].item<double>(), linear_min, linear_max),
        std::clamp(cpu_controls[1].item<double>(), angular_min, angular_max)
    };
}

// 状態推定モデルによる次状態予測（高速化：早期リターン、アクセサ使用）
State_t VelPlanner::predictNextState(const State_t& current_state, double control_linear, double control_angular) const {
    if (!model_loaded_) [[unlikely]] {
        RCLCPP_WARN(rclcpp::get_logger("velplanner"), "Model not loaded. Cannot predict next state.");
        return current_state;
    }
    
    torch::NoGradGuard no_grad;  // 勾配計算無効化
    
    auto input = torch::zeros({1, 1, 7}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    auto input_accessor = input.accessor<float, 3>();
    
    input_accessor[0][0][0] = static_cast<float>(current_state.linear_vel);
    input_accessor[0][0][1] = static_cast<float>(current_state.angular_vel);
    input_accessor[0][0][2] = static_cast<float>(current_state.acc_linear_vel);
    input_accessor[0][0][3] = static_cast<float>(current_state.acc_angular_vel);
    input_accessor[0][0][4] = static_cast<float>(current_state.potentio);
    input_accessor[0][0][5] = static_cast<float>(control_linear);
    input_accessor[0][0][6] = static_cast<float>(control_angular);
    
    auto output = model_.forward({input}).toTensor();
    
    const auto cpu_output = output.device() != torch::kCPU ? output.to(torch::kCPU) : output;
    const auto output_accessor = cpu_output.accessor<float, 2>();
    
    const float dt_inv = 1.0f / mppi_params_.dt;
    State_t next_state;
    next_state.linear_vel = output_accessor[0][0];
    next_state.angular_vel = output_accessor[0][1];
    next_state.potentio = static_cast<int>(output_accessor[0][2]);
    next_state.acc_linear_vel = (next_state.linear_vel - current_state.linear_vel) * dt_inv;
    next_state.acc_angular_vel = (next_state.angular_vel - current_state.angular_vel) * dt_inv;
    
    return next_state;
}

}