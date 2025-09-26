#pragma once

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

namespace yolopnav {

struct VehicleState {
    double x = 0.0;          // x座標 [m]
    double y = 0.0;          // y座標 [m]
    double yaw = 0.0;        // ヨー角 [rad]
    double velocity = 0.0;   // 速度 [m/s]
    double yaw_rate = 0.0;   // ヨー角速度 [rad/s]
};

struct ControlInput {
    double linear_velocity = 0.0;   // 線速度 [m/s]
    double angular_velocity = 0.0;  // 角速度 [rad/s]
};

struct MPCParams {
    // 予測ホライゾン
    double prediction_horizon_time = 2.0;    // 予測時間ホライゾン [s]
    int prediction_steps = 20;               // 予測ステップ数
    double dt = 0.1;                        // 時間刻み [s]
    
    // 車両パラメータ
    double target_velocity = 1.0;           // 目標速度 [m/s]
    double max_linear_velocity = 2.0;       // 最大線速度 [m/s]
    double max_angular_velocity = 1.0;      // 最大角速度 [rad/s]
    double wheelbase = 0.3;                 // ホイールベース [m]
    
    // 横方向探索
    double lateral_search_range = 1.5;      // 横方向探索範囲 [m]
    int lateral_search_points = 11;         // 横方向探索点数
    
    // 重み係数
    double velocity_weight = 1.0;           // 速度制御重み
    double steering_weight = 1.0;           // ステアリング制御重み
    double potential_weight = 10.0;         // ポテンシャル場重み
};

struct PredictedTrajectory {
    std::vector<VehicleState> states;              // 予測状態列
    std::vector<ControlInput> controls;            // 制御入力列
    std::vector<std::vector<Eigen::Vector2d>> lateral_points;  // 各時刻での横方向探索点群
};

class MPCController {
public:
    explicit MPCController(const MPCParams& params = MPCParams());
    
    // メイン処理：現在状態と車線情報からMPCによる最適制御を計算
    ControlInput calculateOptimalControl(const VehicleState& current_state,
                                       const struct LaneData& lane_data);
    
    // 予測軌道の取得（デバッグ用）
    PredictedTrajectory getLastPredictedTrajectory() const;
    
    // パラメータ設定
    void setParams(const MPCParams& params);
    MPCParams getParams() const;

private:
    MPCParams params_;
    PredictedTrajectory last_trajectory_;
    
    // 車両運動モデル
    VehicleState predictNextState(const VehicleState& current_state, 
                                const ControlInput& control_input, 
                                double dt);
    
    // 予測軌道生成
    PredictedTrajectory generatePredictedTrajectory(const VehicleState& initial_state,
                                                  const std::vector<ControlInput>& control_sequence);
    
    // 横方向探索点生成
    std::vector<Eigen::Vector2d> generateLateralSearchPoints(const VehicleState& state);
    
    // コスト関数計算
    double calculateTotalCost(const PredictedTrajectory& trajectory,
                            const struct LaneData& lane_data);
    
    // 速度コスト
    double calculateVelocityCost(const PredictedTrajectory& trajectory);
    
    // ステアリングコスト（制御入力の滑らかさ）
    double calculateSteeringCost(const PredictedTrajectory& trajectory);
    
    // ポテンシャル場コスト
    double calculatePotentialFieldCost(const PredictedTrajectory& trajectory,
                                     const struct LaneData& lane_data);
    
    // 制御入力最適化
    std::vector<ControlInput> optimizeControlSequence(const VehicleState& initial_state,
                                                    const struct LaneData& lane_data);
    
    // 制御入力制約
    ControlInput applyControlConstraints(const ControlInput& control);
    
    // 数値最適化（シンプルなgradient descent）
    std::vector<ControlInput> gradientDescentOptimization(const VehicleState& initial_state,
                                                        const struct LaneData& lane_data,
                                                        std::vector<ControlInput> initial_guess);
    
    // 車線中央への利益計算
    double calculateBenefit(const Eigen::Vector2d& position, 
                          const std::vector<Eigen::Vector3d>& left_points,
                          const std::vector<Eigen::Vector3d>& right_points);
    
    // 車線からのリスク計算
    double calculateRoadRisk(const Eigen::Vector2d& position,
                           const std::vector<Eigen::Vector3d>& lane_points,
                           bool is_left_lane);
};

}  // namespace yolopnav