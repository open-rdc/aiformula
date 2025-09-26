#include "yolopnav/extremum_seeking_mpc.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace yolopnav {

ExtremumSeekingMPC::ExtremumSeekingMPC(const ExtremumSeekingParams& params) 
    : params_(params), current_curvatures_(params.curvatures) {
    
    pose_predictor_ = std::make_unique<PosePredictor>();
    object_risk_calculator_ = std::make_unique<ObjectRiskCalculator>();
    road_risk_calculator_ = std::make_unique<RoadRiskCalculator>();
    
    // 目標速度を設定
    VelocityState velocity{params_.target_velocity, 0.0};
    pose_predictor_->setCurrentVelocity(velocity);
}

void ExtremumSeekingMPC::setParams(const ExtremumSeekingParams& params) {
    params_ = params;
    current_curvatures_ = params.curvatures;
    
    VelocityState velocity{params_.target_velocity, 0.0};
    pose_predictor_->setCurrentVelocity(velocity);
}

ExtremumSeekingParams ExtremumSeekingMPC::getParams() const {
    return params_;
}

void ExtremumSeekingMPC::setPosePredictorParams(const PosePredictorParams& params) {
    pose_predictor_->setParams(params);
}

void ExtremumSeekingMPC::setObjectRiskParams(const ObjectRiskParams& params) {
    object_risk_calculator_->setParams(params);
}

void ExtremumSeekingMPC::setRoadRiskParams(const RoadRiskParams& params) {
    road_risk_calculator_->setParams(params);
}

geometry_msgs::msg::Twist ExtremumSeekingMPC::calculateOptimalControl(const LaneData& lane_data) {
    // ---- aiformula準拠のextremum_seeking_mpc sequence ----
    
    // 1. 自車位置予測と探索点生成
    auto [ego_positions, seek_positions] = predictEgoPosition(current_curvatures_);
    
    // 2. 障害物リスク計算
    auto object_risk = calculateObjectRisk(seek_positions);
    
    // 3. 道路リスク・利益関数計算
    auto [left_road_risk, right_road_risk, benefit] = calculateRoadRisk(seek_positions, lane_data);
    
    // 4. 総合リスク計算
    auto total_risk = calculateTotalRisk(object_risk, left_road_risk, right_road_risk, benefit);
    
    // 5. ヨー角速度計算
    auto [yaw_angle, yaw_rate, new_curvatures] = calculateYawRate(total_risk);
    
    // 6. 線速度計算
    double vehicle_linear_velocity = calculateLinearVelocity(yaw_angle);
    
    // 7. 制御指令作成
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vehicle_linear_velocity;
    cmd_vel.angular.z = yaw_rate;
    
    // 曲率を更新
    current_curvatures_ = new_curvatures;
    
    return cmd_vel;
}

std::pair<std::vector<Eigen::Vector2d>, SeekPositions> ExtremumSeekingMPC::predictEgoPosition(
    const std::vector<double>& curvatures) {
    
    auto ego_positions = pose_predictor_->predictRelativeEgoPositions(curvatures);
    auto seek_position_relative = pose_predictor_->predictRelativeSeekPositions(ego_positions);
    auto seek_positions = pose_predictor_->predictAbsoluteSeekPositions(ego_positions, seek_position_relative);
    
    return std::make_pair(ego_positions, seek_positions);
}

std::vector<std::vector<double>> ExtremumSeekingMPC::calculateObjectRisk(const SeekPositions& seek_positions) {
    return object_risk_calculator_->computeObjectRisk(seek_positions);
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
ExtremumSeekingMPC::calculateRoadRisk(const SeekPositions& seek_positions, const LaneData& lane_data) {
    return road_risk_calculator_->calculateRoadRisk(seek_positions, lane_data);
}

std::vector<std::vector<double>> ExtremumSeekingMPC::calculateTotalRisk(
    const std::vector<std::vector<double>>& object_risk,
    const std::vector<std::vector<double>>& left_road_risk,
    const std::vector<std::vector<double>>& right_road_risk,
    const std::vector<std::vector<double>>& benefit) {
    
    std::vector<std::vector<double>> total_risk;
    
    for (size_t horizon_idx = 0; horizon_idx < object_risk.size(); ++horizon_idx) {
        std::vector<double> horizon_total_risk;
        
        for (size_t point_idx = 0; point_idx < object_risk[horizon_idx].size(); ++point_idx) {
            // aiformula準拠: object + left_road + right_road - benefit_gain * benefit
            double risk = object_risk[horizon_idx][point_idx] +
                         left_road_risk[horizon_idx][point_idx] +
                         right_road_risk[horizon_idx][point_idx] -
                         params_.benefit_gain * benefit[horizon_idx][point_idx];
            
            horizon_total_risk.push_back(risk);
        }
        
        total_risk.push_back(horizon_total_risk);
    }
    
    return total_risk;
}

std::tuple<double, double, std::vector<double>> ExtremumSeekingMPC::calculateYawRate(
    const std::vector<std::vector<double>>& total_risk) {
    
    // Extremum seeking制御による曲率最適化
    auto curvatures = applyExtremumSeekingControl(total_risk);
    
    // 最適曲率を選択
    double optimal_curvature = selectOptimalCurvature(total_risk);
    
    // 最初の予測ホライゾンでの姿勢予測
    auto predicted_pose = pose_predictor_->predictPose(optimal_curvature, 0.5);  // 0.5秒先
    
    double yaw_angle = predicted_pose.yaw;
    double yaw_rate = std::clamp(optimal_curvature * params_.target_velocity, 
                                -params_.max_angular_velocity, 
                                 params_.max_angular_velocity);
    
    return std::make_tuple(yaw_angle, yaw_rate, curvatures);
}

double ExtremumSeekingMPC::calculateLinearVelocity(double yaw_angle) {
    double target_linear_velocity = params_.target_velocity;
    
    // 曲がり角が大きい場合は減速
    double excess_yaw_angle_for_deceleration = std::abs(yaw_angle) - params_.deceleration_angle_maximum;
    if (excess_yaw_angle_for_deceleration > 0.0) {
        target_linear_velocity -= excess_yaw_angle_for_deceleration * params_.deceleration_gain;
        return std::max(target_linear_velocity, 0.0);
    }
    
    return target_linear_velocity;
}

std::vector<double> ExtremumSeekingMPC::applyExtremumSeekingControl(
    const std::vector<std::vector<double>>& total_risk) {
    
    // 簡単な実装：各曲率候補の平均リスクを計算して最適化
    std::vector<double> curvature_risks(current_curvatures_.size(), 0.0);
    
    // 全ホライゾン・全探索点での平均リスクを計算
    for (size_t curv_idx = 0; curv_idx < current_curvatures_.size(); ++curv_idx) {
        double total_risk_sum = 0.0;
        int total_points = 0;
        
        for (const auto& horizon_risk : total_risk) {
            for (double risk : horizon_risk) {
                total_risk_sum += risk;
                total_points++;
            }
        }
        
        if (total_points > 0) {
            curvature_risks[curv_idx] = total_risk_sum / total_points;
        }
    }
    
    // 最小リスクの曲率を中心に新しい候補を生成
    auto min_iter = std::min_element(curvature_risks.begin(), curvature_risks.end());
    size_t min_idx = std::distance(curvature_risks.begin(), min_iter);
    double optimal_curvature = current_curvatures_[min_idx];
    
    // 新しい曲率候補を生成（optimal_curvatureを中心に）
    std::vector<double> new_curvatures = {
        optimal_curvature - 0.05,
        optimal_curvature,
        optimal_curvature + 0.05
    };
    
    return new_curvatures;
}

double ExtremumSeekingMPC::selectOptimalCurvature(const std::vector<std::vector<double>>& total_risk) {
    // 全体での最小リスクを持つ探索点に対応する曲率を推定
    double min_risk = std::numeric_limits<double>::max();
    size_t optimal_point_idx = 0;
    
    for (const auto& horizon_risk : total_risk) {
        for (size_t point_idx = 0; point_idx < horizon_risk.size(); ++point_idx) {
            if (horizon_risk[point_idx] < min_risk) {
                min_risk = horizon_risk[point_idx];
                optimal_point_idx = point_idx;
            }
        }
    }
    
    // 探索点インデックスから曲率を推定（簡単なマッピング）
    double curvature_range = 0.4; // -0.2 to 0.2
    size_t num_points = pose_predictor_->getParams().seek_y_positions[0].size();
    double curvature = -curvature_range/2 + (curvature_range * optimal_point_idx) / (num_points - 1);
    
    return curvature;
}

}  // namespace yolopnav