#include "utilities/velplanner.hpp"
#include "utilities/utils.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace utils;

namespace velplanner{

rclcpp::Clock system_clock(RCL_ROS_TIME);
int64_t micros(){
    return system_clock.now().nanoseconds()*1e-3;
}

// コンストラクタ
VelPlanner::VelPlanner() 
    : rng_(std::random_device{}()), noise_dist_(0.0, 1.0) {
    initializeTensors();
}

// モデルファイルを読み込み
bool VelPlanner::loadModel(const std::string& model_path) {
    try {
        model_ = torch::jit::load(model_path);
        model_.eval();
        model_loaded_ = true;
        RCLCPP_INFO(rclcpp::get_logger("velplanner"), "MPPI model loaded successfully: %s", model_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("velplanner"), "Failed to load MPPI model: %s", e.what());
        return false;
    }
}

// テンソルの事前初期化（メモリ効率向上）
void VelPlanner::initializeTensors() {
    // GPU利用可能かチェック
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    
    // 事前にテンソルを確保
    samples_tensor_ = torch::zeros({mppi_params_.num_samples, mppi_params_.horizon, 2}, 
                                   torch::TensorOptions().dtype(torch::kFloat32).device(device));
    states_tensor_ = torch::zeros({mppi_params_.num_samples, mppi_params_.horizon + 1, 7}, 
                                  torch::TensorOptions().dtype(torch::kFloat32).device(device));
    costs_tensor_ = torch::zeros({mppi_params_.num_samples}, 
                                 torch::TensorOptions().dtype(torch::kFloat32).device(device));
    
    RCLCPP_INFO(rclcpp::get_logger("velplanner"), "MPPI tensors initialized on %s", 
                device.is_cuda() ? "CUDA" : "CPU");
}

// 目標速度の設定
void VelPlanner::setTargetVelocity(double target_linear, double target_angular) {
    target_linear_ = target_linear;
    target_angular_ = target_angular;
}

// メインの制御サイクル
void VelPlanner::cycle(const State_t state) {
    current_state_ = state;
    
    // 状態バッファの更新
    std::array<float, 7> input = {
        static_cast<float>(state.linear_vel),
        static_cast<float>(state.angular_vel),
        static_cast<float>(state.acc_linear_vel),
        static_cast<float>(state.acc_angular_vel),
        static_cast<float>(state.potentio),
        static_cast<float>(optimal_linear_),  // 前回の制御入力
        static_cast<float>(optimal_angular_)
    };
    
    state_buffer_.push_back(input);
    if (state_buffer_.size() > static_cast<size_t>(seq_len_)) {
        state_buffer_.pop_front();
    }
    
    // モデルが読み込まれており、十分な履歴がある場合にMPPI制御実行
    if (model_loaded_ && state_buffer_.size() == static_cast<size_t>(seq_len_)) {
        // 制御サンプルの生成
        torch::Tensor control_samples = generateControlSamples();
        
        // モデルロールアウト
        torch::Tensor predicted_states = rolloutModel(control_samples);
        
        // コスト計算
        torch::Tensor costs = computeCosts(predicted_states, control_samples);
        
        // 最適制御の計算
        auto optimal_control = computeOptimalControl(control_samples, costs);
        optimal_linear_ = optimal_control.first;
        optimal_angular_ = optimal_control.second;
    } else if (!model_loaded_) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("velplanner"), system_clock, 5000,
                             "MPPI model not loaded. Call loadModel() first.");
    }
}

// 制御サンプルの生成（高速化のため事前確保したテンソル使用）
torch::Tensor VelPlanner::generateControlSamples() const {
    // ノイズ生成
    torch::Tensor noise = torch::randn_like(samples_tensor_);
    
    // ノイズのスケーリング
    noise.select(2, 0) *= mppi_params_.noise_sigma_linear;   // 線速度ノイズ
    noise.select(2, 1) *= mppi_params_.noise_sigma_angular;  // 角速度ノイズ
    
    // 制約の適用
    torch::Tensor control_samples = noise.clone();
    control_samples.select(2, 0).clamp_(-1.0, 1.0);   // 線速度制約
    control_samples.select(2, 1).clamp_(-2.0, 2.0);   // 角速度制約
    
    return control_samples;
}

// モデルロールアウト（最適化されたバッチ処理）
torch::Tensor VelPlanner::rolloutModel(const torch::Tensor& control_sequences) const {
    const int num_samples = control_sequences.size(0);
    const int horizon = control_sequences.size(1);
    
    torch::Device device = control_sequences.device();
    
    // 事前確保されたテンソルを再利用
    torch::Tensor predicted_states = states_tensor_.narrow(0, 0, num_samples).narrow(1, 0, horizon + 1);
    predicted_states.zero_();
    
    // 初期状態の設定（ベクトル化）
    predicted_states.select(1, 0).select(1, 0).fill_(current_state_.linear_vel);
    predicted_states.select(1, 0).select(1, 1).fill_(current_state_.angular_vel);
    predicted_states.select(1, 0).select(1, 2).fill_(current_state_.acc_linear_vel);
    predicted_states.select(1, 0).select(1, 3).fill_(current_state_.acc_angular_vel);
    predicted_states.select(1, 0).select(1, 4).fill_(current_state_.potentio);
    
    // 状態バッファをテンソルに変換（一度だけ）
    torch::Tensor base_state_buffer = torch::zeros({seq_len_, 7}, 
                                                    torch::TensorOptions().dtype(torch::kFloat32).device(device));
    for (int j = 0; j < seq_len_ && j < static_cast<int>(state_buffer_.size()); ++j) {
        const auto& state = state_buffer_[j];
        for (int k = 0; k < 7; ++k) {
            base_state_buffer[j][k] = state[k];
        }
    }
    
    // バッチ入力テンソルの準備（全サンプル共通部分）
    torch::Tensor batch_input = base_state_buffer.unsqueeze(0).expand({num_samples, seq_len_, 7}).clone();
    
    // 時間方向のロールアウト（並列処理対応）
    torch::NoGradGuard no_grad;  // 勾配計算を無効化して高速化
    
    for (int t = 0; t < horizon; ++t) {
        // 制御入力を最新ステップに設定
        batch_input.select(1, seq_len_ - 1).select(1, 5) = control_sequences.select(1, t).select(1, 0);
        batch_input.select(1, seq_len_ - 1).select(1, 6) = control_sequences.select(1, t).select(1, 1);
        
        // バッチ推論（JIT最適化）
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(batch_input);
        
        torch::Tensor output = model_.forward(inputs).toTensor();
        
        // 次の状態を更新（インプレース操作で高速化）
        predicted_states.select(1, t + 1).select(1, 0).copy_(output.select(1, 0));  // v1
        predicted_states.select(1, t + 1).select(1, 1).copy_(output.select(1, 1));  // w1
        predicted_states.select(1, t + 1).select(1, 4).copy_(output.select(1, 2));  // potentio_th1
        
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
        
        // 次のステップ用に状態バッファを更新（スライディングウィンドウ）
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

// コスト計算（ベクトル化で高速化）
torch::Tensor VelPlanner::computeCosts(const torch::Tensor& predicted_states, 
                                      const torch::Tensor& control_sequences) const {
    const int num_samples = predicted_states.size(0);
    const int horizon = predicted_states.size(1) - 1;
    
    // 事前確保されたテンソルを再利用
    torch::Tensor costs = costs_tensor_.narrow(0, 0, num_samples);
    costs.zero_();
    
    // 速度誤差コスト（全時刻まとめて計算）
    torch::Tensor vel_states = predicted_states.narrow(1, 1, horizon);  // t=1からhorizonまで
    torch::Tensor linear_error = torch::pow(vel_states.select(2, 0) - target_linear_, 2);
    torch::Tensor angular_error = torch::pow(vel_states.select(2, 1) - target_angular_, 2);
    torch::Tensor vel_error_sum = torch::sum(linear_error + angular_error, 1);
    costs += mppi_params_.weight_vel_error * vel_error_sum;
    
    // 制御入力コスト（全時刻まとめて計算）
    torch::Tensor control_squared = torch::pow(control_sequences, 2);
    torch::Tensor control_cost_sum = torch::sum(control_squared.select(2, 0) + control_squared.select(2, 1), 1);
    costs += mppi_params_.weight_control_effort * control_cost_sum;
    
    // 滑らかさコスト（制御入力の変化率、ベクトル化）
    if (horizon > 1) {
        torch::Tensor control_diff = control_sequences.narrow(1, 1, horizon - 1) - 
                                    control_sequences.narrow(1, 0, horizon - 1);
        torch::Tensor smoothness_squared = torch::pow(control_diff, 2);
        torch::Tensor smoothness_sum = torch::sum(smoothness_squared.select(2, 0) + smoothness_squared.select(2, 1), 1);
        costs += mppi_params_.weight_smoothness * smoothness_sum;
    }
    
    return costs;
}

// 最適制御の計算（重み付き平均、高速化版）
std::pair<double, double> VelPlanner::computeOptimalControl(const torch::Tensor& control_samples,
                                                           const torch::Tensor& costs) const {
    // 重みの計算（指数関数的重み付け）
    torch::Tensor weights = torch::exp(-costs / mppi_params_.lambda);
    torch::Tensor weight_sum = torch::sum(weights);
    weights = weights / weight_sum;
    
    // 最初のステップの制御入力のみ計算（receding horizon）
    torch::Tensor first_step_controls = control_samples.select(1, 0);  // [num_samples, 2]
    
    // 重み付き平均の計算（効率化）
    torch::Tensor weighted_controls = weights.unsqueeze(1) * first_step_controls;  // [num_samples, 2]
    torch::Tensor optimal_controls = torch::sum(weighted_controls, 0);  // [2]
    
    double optimal_linear = optimal_controls[0].item<double>();
    double optimal_angular = optimal_controls[1].item<double>();
    
    // 制約の適用
    optimal_linear = std::clamp(optimal_linear, -1.0, 1.0);
    optimal_angular = std::clamp(optimal_angular, -2.0, 2.0);
    
    return {optimal_linear, optimal_angular};
}

// 状態推定モデルによる次状態予測
State_t VelPlanner::predictNextState(const State_t& current_state, double control_linear, double control_angular) const {
    if (!model_loaded_) {
        RCLCPP_WARN(rclcpp::get_logger("velplanner"), "Model not loaded. Cannot predict next state.");
        return current_state;
    }
    
    torch::NoGradGuard no_grad;  // 勾配計算無効化
    
    // 入力テンソルの準備 [1, 1, 7] (batch_size=1, seq_len=1, feature_dim=7)
    torch::Tensor input = torch::zeros({1, 1, 7}, torch::TensorOptions().dtype(torch::kFloat32));
    
    input[0][0][0] = current_state.linear_vel;        // v0
    input[0][0][1] = current_state.angular_vel;       // w0
    input[0][0][2] = current_state.acc_linear_vel;    // acc_v0
    input[0][0][3] = current_state.acc_angular_vel;   // acc_w0
    input[0][0][4] = current_state.potentio;          // potentio_th0
    input[0][0][5] = control_linear;                  // linear_x (制御入力)
    input[0][0][6] = control_angular;                 // angular_z (制御入力)
    
    // GPU/CPUに配置
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    input = input.to(device);
    
    // モデル推論
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input);
    
    torch::Tensor output = model_.forward(inputs).toTensor();
    
    // CPUに戻す
    output = output.to(torch::kCPU);
    
    // 次状態の構築
    State_t next_state;
    next_state.linear_vel = output[0][0].item<float>();      // v1
    next_state.angular_vel = output[0][1].item<float>();     // w1
    next_state.potentio = static_cast<int>(output[0][2].item<float>());  // potentio_th1
    
    // 加速度の数値微分計算
    next_state.acc_linear_vel = (next_state.linear_vel - current_state.linear_vel) / mppi_params_.dt;
    next_state.acc_angular_vel = (next_state.angular_vel - current_state.angular_vel) / mppi_params_.dt;
    
    return next_state;
}

}