#include "yolopnav/mpc_controller.hpp"
#include "yolopnav/potential_field_generator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace yolopnav {

MPCController::MPCController(const MPCParams& params) : params_(params) {
    params_.dt = params_.prediction_horizon_time / params_.prediction_steps;
}

void MPCController::setParams(const MPCParams& params) {
    params_ = params;
    params_.dt = params_.prediction_horizon_time / params_.prediction_steps;
}

MPCParams MPCController::getParams() const {
    return params_;
}

ControlInput MPCController::calculateOptimalControl(const VehicleState& current_state,
                                                  const LaneData& lane_data) {
    // 制御入力列を最適化
    auto optimal_controls = optimizeControlSequence(current_state, lane_data);
    
    // 最初の制御入力を返す（MPCのreceding horizon principle）
    if (!optimal_controls.empty()) {
        last_trajectory_ = generatePredictedTrajectory(current_state, optimal_controls);
        return optimal_controls[0];
    }
    
    // フォールバック：停止
    return ControlInput{0.0, 0.0};
}

PredictedTrajectory MPCController::getLastPredictedTrajectory() const {
    return last_trajectory_;
}

VehicleState MPCController::predictNextState(const VehicleState& current_state,
                                           const ControlInput& control_input,
                                           double dt) {
    VehicleState next_state;
    
    // Bicycle model (簡単な車両モデル)
    double v = control_input.linear_velocity;
    double omega = control_input.angular_velocity;
    
    next_state.x = current_state.x + v * std::cos(current_state.yaw) * dt;
    next_state.y = current_state.y + v * std::sin(current_state.yaw) * dt;
    next_state.yaw = current_state.yaw + omega * dt;
    next_state.velocity = v;
    next_state.yaw_rate = omega;
    
    return next_state;
}

PredictedTrajectory MPCController::generatePredictedTrajectory(const VehicleState& initial_state,
                                                             const std::vector<ControlInput>& control_sequence) {
    PredictedTrajectory trajectory;
    VehicleState current_state = initial_state;
    
    trajectory.states.push_back(current_state);
    
    for (size_t i = 0; i < control_sequence.size(); ++i) {
        trajectory.controls.push_back(control_sequence[i]);
        
        // 次状態を予測
        current_state = predictNextState(current_state, control_sequence[i], params_.dt);
        trajectory.states.push_back(current_state);
        
        // 横方向探索点を生成
        auto lateral_points = generateLateralSearchPoints(current_state);
        trajectory.lateral_points.push_back(lateral_points);
    }
    
    return trajectory;
}

std::vector<Eigen::Vector2d> MPCController::generateLateralSearchPoints(const VehicleState& state) {
    std::vector<Eigen::Vector2d> points;
    
    double step = 2.0 * params_.lateral_search_range / (params_.lateral_search_points - 1);
    
    for (int i = 0; i < params_.lateral_search_points; ++i) {
        double lateral_offset = -params_.lateral_search_range + i * step;
        
        // 車両の向きに垂直な方向にオフセット
        double offset_x = -lateral_offset * std::sin(state.yaw);
        double offset_y = lateral_offset * std::cos(state.yaw);
        
        points.emplace_back(state.x + offset_x, state.y + offset_y);
    }
    
    return points;
}

double MPCController::calculateTotalCost(const PredictedTrajectory& trajectory,
                                       const LaneData& lane_data) {
    double velocity_cost = calculateVelocityCost(trajectory);
    double steering_cost = calculateSteeringCost(trajectory);
    double potential_cost = calculatePotentialFieldCost(trajectory, lane_data);
    
    return params_.velocity_weight * velocity_cost +
           params_.steering_weight * steering_cost +
           params_.potential_weight * potential_cost;
}

double MPCController::calculateVelocityCost(const PredictedTrajectory& trajectory) {
    double cost = 0.0;
    
    for (const auto& control : trajectory.controls) {
        double velocity_error = control.linear_velocity - params_.target_velocity;
        cost += velocity_error * velocity_error;
    }
    
    return cost;
}

double MPCController::calculateSteeringCost(const PredictedTrajectory& trajectory) {
    double cost = 0.0;
    
    // 制御入力の変化量を最小化（滑らかな制御）
    for (size_t i = 1; i < trajectory.controls.size(); ++i) {
        double angular_diff = trajectory.controls[i].angular_velocity - 
                             trajectory.controls[i-1].angular_velocity;
        cost += angular_diff * angular_diff;
    }
    
    return cost;
}

double MPCController::calculatePotentialFieldCost(const PredictedTrajectory& trajectory,
                                                const LaneData& lane_data) {
    double total_cost = 0.0;
    
    // 各予測時刻での横方向探索点でポテンシャル場を評価
    for (const auto& lateral_points : trajectory.lateral_points) {
        double min_cost = std::numeric_limits<double>::max();
        
        for (const auto& point : lateral_points) {
            // 利益（車線中央への引力）
            double benefit = calculateBenefit(point, lane_data.left_points, lane_data.right_points);
            
            // リスク（車線からの斥力）
            double left_risk = calculateRoadRisk(point, lane_data.left_points, true);
            double right_risk = calculateRoadRisk(point, lane_data.right_points, false);
            
            // 総コスト（リスクが高いほど、利益が低いほどコストが高い）
            double point_cost = left_risk + right_risk - benefit;
            min_cost = std::min(min_cost, point_cost);
        }
        
        total_cost += min_cost;
    }
    
    return total_cost;
}

double MPCController::calculateBenefit(const Eigen::Vector2d& position,
                                     const std::vector<Eigen::Vector3d>& left_points,
                                     const std::vector<Eigen::Vector3d>& right_points) {
    if (left_points.empty() || right_points.empty()) {
        return 0.0;
    }
    
    // 最も近い左右の点を見つけて中央を計算
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d center_point;
    
    for (size_t i = 0; i < std::min(left_points.size(), right_points.size()); ++i) {
        Eigen::Vector2d left_2d(left_points[i].x(), left_points[i].y());
        Eigen::Vector2d right_2d(right_points[i].x(), right_points[i].y());
        Eigen::Vector2d center = (left_2d + right_2d) * 0.5;
        
        double distance = (position - center).norm();
        if (distance < min_distance) {
            min_distance = distance;
            center_point = center;
        }
    }
    
    // ガウシアンによる利益関数
    double sigma = 0.5; // 標準偏差
    double distance_to_center = (position - center_point).norm();
    return std::exp(-0.5 * distance_to_center * distance_to_center / (sigma * sigma));
}

double MPCController::calculateRoadRisk(const Eigen::Vector2d& position,
                                      const std::vector<Eigen::Vector3d>& lane_points,
                                      bool is_left_lane) {
    if (lane_points.empty()) {
        return 0.0;
    }
    
    // 最も近い車線点を見つける
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& point : lane_points) {
        Eigen::Vector2d lane_2d(point.x(), point.y());
        double distance = (position - lane_2d).norm();
        min_distance = std::min(min_distance, distance);
    }
    
    // 車線に近づくほどリスクが高くなる
    double risk_distance = 0.3; // リスクが発生する距離
    if (min_distance < risk_distance) {
        double risk_ratio = (risk_distance - min_distance) / risk_distance;
        double base_risk = risk_ratio * risk_ratio; // 二次関数的にリスク増加
        
        // 左右の車線で異なるリスク重み（将来の拡張用）
        double lane_weight = is_left_lane ? 1.0 : 1.0;
        return base_risk * lane_weight;
    }
    
    return 0.0;
}

std::vector<ControlInput> MPCController::optimizeControlSequence(const VehicleState& initial_state,
                                                               const LaneData& lane_data) {
    // 初期推定値（前進のみ）
    std::vector<ControlInput> initial_guess(params_.prediction_steps);
    for (auto& control : initial_guess) {
        control.linear_velocity = params_.target_velocity;
        control.angular_velocity = 0.0;
    }
    
    // 勾配降下法による最適化
    return gradientDescentOptimization(initial_state, lane_data, initial_guess);
}

std::vector<ControlInput> MPCController::gradientDescentOptimization(
    const VehicleState& initial_state,
    const LaneData& lane_data,
    std::vector<ControlInput> initial_guess) {
    
    std::vector<ControlInput> current_controls = initial_guess;
    double learning_rate = 0.01;
    int max_iterations = 10; // リアルタイム性を考慮して少なめ
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 現在のコストを計算
        auto trajectory = generatePredictedTrajectory(initial_state, current_controls);
        double current_cost = calculateTotalCost(trajectory, lane_data);
        
        // 各制御入力に対する勾配を数値的に計算
        std::vector<ControlInput> gradient(params_.prediction_steps);
        double epsilon = 0.01;
        
        for (size_t i = 0; i < current_controls.size(); ++i) {
            // 線速度の勾配
            auto controls_plus = current_controls;
            controls_plus[i].linear_velocity += epsilon;
            auto traj_plus = generatePredictedTrajectory(initial_state, controls_plus);
            double cost_plus = calculateTotalCost(traj_plus, lane_data);
            gradient[i].linear_velocity = (cost_plus - current_cost) / epsilon;
            
            // 角速度の勾配
            auto controls_plus_angular = current_controls;
            controls_plus_angular[i].angular_velocity += epsilon;
            auto traj_plus_angular = generatePredictedTrajectory(initial_state, controls_plus_angular);
            double cost_plus_angular = calculateTotalCost(traj_plus_angular, lane_data);
            gradient[i].angular_velocity = (cost_plus_angular - current_cost) / epsilon;
        }
        
        // 制御入力を更新
        for (size_t i = 0; i < current_controls.size(); ++i) {
            current_controls[i].linear_velocity -= learning_rate * gradient[i].linear_velocity;
            current_controls[i].angular_velocity -= learning_rate * gradient[i].angular_velocity;
            
            // 制約を適用
            current_controls[i] = applyControlConstraints(current_controls[i]);
        }
    }
    
    return current_controls;
}

ControlInput MPCController::applyControlConstraints(const ControlInput& control) {
    ControlInput constrained = control;
    
    // 速度制約
    constrained.linear_velocity = std::max(0.0, 
        std::min(params_.max_linear_velocity, constrained.linear_velocity));
    
    // 角速度制約
    constrained.angular_velocity = std::max(-params_.max_angular_velocity,
        std::min(params_.max_angular_velocity, constrained.angular_velocity));
    
    return constrained;
}

}  // namespace yolopnav