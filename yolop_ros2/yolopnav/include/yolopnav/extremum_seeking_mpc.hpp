#pragma once

#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>

#include "yolopnav/pose_predictor.hpp"
#include "yolopnav/object_risk_calculator.hpp"
#include "yolopnav/potential_field_generator.hpp"

namespace yolopnav {

struct ExtremumSeekingParams {
    // 予測パラメータ
    std::vector<double> curvatures = {-0.2, 0.0, 0.2};  // 初期曲率候補
    double benefit_gain = 2.0;                          // 利益関数のゲイン
    
    // 制御パラメータ
    double target_velocity = 1.6;                       // 目標速度[m/s]
    double max_angular_velocity = 0.32;                 // 最大角速度[rad/s] (curvature_limit * target_velocity)
    double deceleration_angle_maximum = 0.5;            // 減速開始角度[rad]
    double deceleration_gain = 1.0;                     // 減速ゲイン
};

class ExtremumSeekingMPC {
public:
    explicit ExtremumSeekingMPC(const ExtremumSeekingParams& params = ExtremumSeekingParams());
    
    // aiformula準拠のメイン処理シーケンス
    geometry_msgs::msg::Twist calculateOptimalControl(const LaneData& lane_data);
    
    // 個別処理関数群（aiformula準拠）
    std::pair<std::vector<Eigen::Vector2d>, SeekPositions> predictEgoPosition(const std::vector<double>& curvatures);
    
    std::vector<std::vector<double>> calculateObjectRisk(const SeekPositions& seek_positions);
    
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
    calculateRoadRisk(const SeekPositions& seek_positions, const LaneData& lane_data);
    
    std::vector<std::vector<double>> calculateTotalRisk(const std::vector<std::vector<double>>& object_risk,
                                                       const std::vector<std::vector<double>>& left_road_risk,
                                                       const std::vector<std::vector<double>>& right_road_risk,
                                                       const std::vector<std::vector<double>>& benefit);
    
    std::tuple<double, double, std::vector<double>> calculateYawRate(const std::vector<std::vector<double>>& total_risk);
    
    double calculateLinearVelocity(double yaw_angle);
    
    // パラメータ設定
    void setParams(const ExtremumSeekingParams& params);
    ExtremumSeekingParams getParams() const;
    
    // サブコンポーネントのパラメータ設定
    void setPosePredictorParams(const PosePredictorParams& params);
    void setObjectRiskParams(const ObjectRiskParams& params);
    void setRoadRiskParams(const RoadRiskParams& params);

private:
    ExtremumSeekingParams params_;
    std::vector<double> current_curvatures_;
    
    // サブコンポーネント
    std::unique_ptr<PosePredictor> pose_predictor_;
    std::unique_ptr<ObjectRiskCalculator> object_risk_calculator_;
    std::unique_ptr<RoadRiskCalculator> road_risk_calculator_;
    
    // Extremum seeking optimization (簡単な実装)
    std::vector<double> applyExtremumSeekingControl(const std::vector<std::vector<double>>& total_risk);
    
    // 最小リスク曲率の選択
    double selectOptimalCurvature(const std::vector<std::vector<double>>& total_risk);
};

}  // namespace yolopnav