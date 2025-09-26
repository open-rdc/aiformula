#include "yolopnav/pose_predictor.hpp"
#include <cmath>
#include <algorithm>

namespace yolopnav {

PosePredictor::PosePredictor(const PosePredictorParams& params) 
    : params_(params), ego_current_velocity_{1.0, 0.0} {
    // デフォルト速度を設定
}

void PosePredictor::setCurrentVelocity(const VelocityState& velocity) {
    ego_current_velocity_ = velocity;
}

void PosePredictor::setParams(const PosePredictorParams& params) {
    params_ = params;
}

PosePredictorParams PosePredictor::getParams() const {
    return params_;
}

std::vector<Eigen::Vector2d> PosePredictor::predictRelativeEgoPositions(const std::vector<double>& curvatures) {
    std::vector<Eigen::Vector2d> ego_positions;
    
    for (int horizon_idx = 0; horizon_idx < params_.horizon_length; ++horizon_idx) {
        double curvature = (horizon_idx < static_cast<int>(curvatures.size())) ? 
                          curvatures[horizon_idx] : 0.0;
        double horizon_time = params_.horizon_times[horizon_idx];
        
        // 各ホライゾンでの予測位置を計算
        Pose predicted_pose = predictPose(curvature, horizon_time);
        ego_positions.emplace_back(predicted_pose.pos.x, predicted_pose.pos.y);
    }
    
    return ego_positions;
}

SeekPositions PosePredictor::predictRelativeSeekPositions(const std::vector<Eigen::Vector2d>& ego_positions) {
    SeekPositions relative_seek_positions(params_.horizon_length);
    
    for (int horizon_idx = 0; horizon_idx < params_.horizon_length; ++horizon_idx) {
        // x座標（すべて0 - 相対位置なので）
        std::vector<double> x_positions(params_.seek_y_positions[horizon_idx].size(), 0.0);
        
        // y座標（横方向探索点）
        std::vector<double> y_positions = params_.seek_y_positions[horizon_idx];
        
        // [x, y]の形で格納
        relative_seek_positions[horizon_idx] = {x_positions, y_positions};
    }
    
    return relative_seek_positions;
}

SeekPositions PosePredictor::predictAbsoluteSeekPositions(const std::vector<Eigen::Vector2d>& ego_positions,
                                                        const SeekPositions& relative_seek_positions) {
    SeekPositions absolute_seek_positions(params_.horizon_length);
    
    Eigen::Vector2d prev_position = ego_positions[0];
    
    for (int horizon_idx = 0; horizon_idx < params_.horizon_length; ++horizon_idx) {
        const auto& relative_pos = relative_seek_positions[horizon_idx];
        
        if (horizon_idx == 0) {
            // 最初のホライゾンは相対位置をそのまま使用
            absolute_seek_positions[horizon_idx] = relative_pos;
        } else {
            // 累積位置 + 相対位置
            Eigen::Vector2d cumulative_ego_pos = prev_position + ego_positions[horizon_idx];
            
            std::vector<double> abs_x_positions, abs_y_positions;
            
            for (size_t point_idx = 0; point_idx < relative_pos[0].size(); ++point_idx) {
                abs_x_positions.push_back(cumulative_ego_pos.x() + relative_pos[0][point_idx]);
                abs_y_positions.push_back(cumulative_ego_pos.y() + relative_pos[1][point_idx]);
            }
            
            absolute_seek_positions[horizon_idx] = {abs_x_positions, abs_y_positions};
            prev_position = ego_positions[horizon_idx];
        }
    }
    
    return absolute_seek_positions;
}

Pose PosePredictor::predictPose(double curvature, double horizon_time) {
    double max_curvature = 1.0 / params_.curvature_radius_maximum;
    double radius = (std::abs(curvature) > max_curvature) ? 
                   (1.0 / curvature) : params_.curvature_radius_maximum;
    
    double travel_distance = ego_current_velocity_.linear * horizon_time;
    double arc_angle = travel_distance / radius;
    
    double x, y;
    if (radius < params_.curvature_radius_maximum) {
        // カーブ
        x = radius * std::sin(arc_angle);
        y = radius * (1.0 - std::cos(arc_angle));
    } else {
        // 直線
        x = travel_distance;
        y = 0.0;
        arc_angle = 0.0;
    }
    
    return Pose(Position2d(x, y), arc_angle);
}

Eigen::Matrix2d PosePredictor::createRotationMatrix(double angle) {
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << std::cos(angle), -std::sin(angle),
                       std::sin(angle),  std::cos(angle);
    return rotation_matrix;
}

Eigen::Vector2d PosePredictor::rotatePosition(const Position2d& position, double angle) {
    Eigen::Matrix2d rotation_matrix = createRotationMatrix(angle);
    return rotation_matrix * position.as_array();
}

}  // namespace yolopnav