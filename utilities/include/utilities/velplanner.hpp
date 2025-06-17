#pragma once

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <array>
#include <vector>
#include <random>
#include <memory>

#include <torch/torch.h>
#include <torch/script.h>

namespace velplanner{

struct Physics_t{
    Physics_t(){}
    Physics_t(double pos, double vel, double acc):pos(pos), vel(vel), acc(acc){}
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
};

struct State_t{
    State_t(){}
    State_t(double linear_vel, double angular_vel, double acc_linear_vel, double acc_angular_vel, int potentio)
    : linear_vel(linear_vel), angular_vel(angular_vel), acc_linear_vel(acc_linear_vel), acc_angular_vel(acc_angular_vel), potentio(potentio){}
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    double acc_linear_vel = 0.0;
    double acc_angular_vel = 0.0;
    int potentio = 0;
};

// MPPI制御用パラメータ構造体
struct MPPIParams {
    int num_samples = 1000;           // サンプル数
    int horizon = 20;                 // 予測ホライゾン
    double lambda = 1.0;              // 温度パラメータ
    double noise_sigma_linear = 0.3;  // 線速度ノイズ標準偏差
    double noise_sigma_angular = 0.5; // 角速度ノイズ標準偏差
    double dt = 0.05;                 // 時間ステップ[s]
    
    // 重み係数
    double weight_vel_error = 1.0;    // 速度誤差重み
    double weight_control_effort = 0.1; // 制御入力重み
    double weight_smoothness = 0.05;  // 滑らかさ重み
};

class VelPlanner{
public:
    VelPlanner();
    ~VelPlanner() = default;
    
    // モデルファイルを読み込み
    bool loadModel(const std::string& model_path);
    
    // MPPI制御で最適制御入力を計算
    void cycle(const State_t current_state);
    void setTargetVelocity(double target_linear, double target_angular);
    
    // MPPI制御結果の取得
    std::pair<double, double> getOptimalControl() const { return {optimal_linear_, optimal_angular_}; }
    
    // 状態推定モデルによる次状態予測
    State_t predictNextState(const State_t& current_state, double control_linear, double control_angular) const;
    
    // MPPI パラメータの設定
    void setMPPIParams(const MPPIParams& params) { mppi_params_ = params; initializeTensors(); }
    const MPPIParams& getMPPIParams() const { return mppi_params_; }

private:
    // 基本パラメータ
    State_t current_state_;
    mutable torch::jit::script::Module model_;
    bool model_loaded_ = false;

    const int seq_len_ = 10;
    std::deque<std::array<float, 7>> state_buffer_;
    
    // MPPI制御用メンバ
    MPPIParams mppi_params_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
    
    // 目標速度
    double target_linear_ = 0.0;
    double target_angular_ = 0.0;
    
    // 最適制御入力
    double optimal_linear_ = 0.0;
    double optimal_angular_ = 0.0;
    
    // 事前確保されたメモリ
    mutable torch::Tensor samples_tensor_;
    mutable torch::Tensor states_tensor_;
    mutable torch::Tensor costs_tensor_;
    
    // MPPI制御用プライベート関数
    void initializeTensors();
    torch::Tensor rolloutModel(const torch::Tensor& control_sequences) const;
    torch::Tensor computeCosts(const torch::Tensor& predicted_states, 
                              const torch::Tensor& control_sequences) const;
    torch::Tensor generateControlSamples() const;
    std::pair<double, double> computeOptimalControl(const torch::Tensor& control_samples,
                                                   const torch::Tensor& costs) const;
};

//alias
using Limit = Physics_t;

}