#pragma once

#include <array>
#include <vector>

namespace trajectory_follower
{

// 横方向 MPC のパラメータ。
// Autoware mpc_lateral_controller の運動学自転車誤差モデル(ステア1次遅れ)に基づく。
struct LateralMpcParams
{
    double wheelbase = 0.8;                    // ホイールベース L [m]
    double steer_tau = 0.3;                    // ステア1次遅れ時定数 τ [s]
    double steer_limit = 0.2618;               // ステア角上下限 [rad]
    double weight_lat_error = 1.0;             // 横偏差重み
    double weight_heading_error = 1.0;         // 方位偏差重み
    double weight_steering_input = 0.5;        // ステア入力重み R
    double weight_steer_rate = 5.0;            // ステアレート重み Rd
    double weight_terminal_lat_error = 1.0;    // 終端 横偏差重み
    double weight_terminal_heading_error = 1.0;// 終端 方位偏差重み
    int horizon = 20;                          // 予測ステップ数 N
    double prediction_dt = 0.1;                // 予測周期 [s]
    double min_predict_speed = 0.5;            // 予測に用いる最小速度 [m/s]
};

// 運動学自転車誤差モデル + condensed QP による横方向 MPC (ROS 非依存)。
// 状態 x = [横偏差 e_y, 方位偏差 e_yaw, ステア δ]、入力 u = ステア指令。
//   ė_y   = V e_yaw
//   ė_yaw = (V/L) δ − V κ
//   δ̇     = −(1/τ) δ + (1/τ) u
// QP は Eigen の最小二乗(無制約)で解き、出力をステア上下限でクランプする
// (Autoware の unconstraint_fast 相当)。
class LateralMpc
{
public:
    void configure(const LateralMpcParams & params);

    // base_link frame の経路点列 (自車は原点・前進 +x)、参照速度 v [m/s] から
    // ステア指令 [rad] を計算する。
    double computeSteering(const std::vector<std::array<double, 2>> & path_xy, double v);

    void reset();

    double previousSteer() const { return prev_steer_; }

private:
    LateralMpcParams p_{};
    double prev_steer_ = 0.0;
};

}  // namespace trajectory_follower
