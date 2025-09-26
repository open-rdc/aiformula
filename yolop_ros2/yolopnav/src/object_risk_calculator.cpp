#include "yolopnav/object_risk_calculator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace yolopnav {

ObjectRiskCalculator::ObjectRiskCalculator(const ObjectRiskParams& params) 
    : params_(params) {}

void ObjectRiskCalculator::setParams(const ObjectRiskParams& params) {
    params_ = params;
}

ObjectRiskParams ObjectRiskCalculator::getParams() const {
    return params_;
}

void ObjectRiskCalculator::setDetectedObjects(const std::vector<Eigen::Vector2d>& object_positions) {
    detected_objects_ = object_positions;
}

std::vector<std::vector<double>> ObjectRiskCalculator::computeObjectRisk(const SeekPositions& seek_positions) {
    std::vector<std::vector<double>> object_risks;
    
    for (const auto& horizon_positions : seek_positions) {
        std::vector<double> horizon_risks;
        
        // 各探索点でのリスクを計算
        for (size_t point_idx = 0; point_idx < horizon_positions[0].size(); ++point_idx) {
            Eigen::Vector2d seek_point(horizon_positions[0][point_idx], 
                                     horizon_positions[1][point_idx]);
            
            double risk = calculatePointRisk(seek_point);
            horizon_risks.push_back(risk);
        }
        
        object_risks.push_back(horizon_risks);
    }
    
    return object_risks;
}

double ObjectRiskCalculator::calculatePointRisk(const Eigen::Vector2d& position) {
    if (!params_.enable_object_detection || detected_objects_.empty()) {
        return 0.0;  // 障害物検出が無効または障害物がない場合
    }
    
    double min_distance = calculateMinDistanceToObjects(position);
    
    // 距離に基づくリスク計算
    if (min_distance < params_.risk_distance) {
        double distance_ratio = (params_.risk_distance - min_distance) / params_.risk_distance;
        return params_.risk_gain * distance_ratio * distance_ratio;
    }
    
    return 0.0;
}

double ObjectRiskCalculator::calculateMinDistanceToObjects(const Eigen::Vector2d& position) {
    if (detected_objects_.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& object_pos : detected_objects_) {
        double distance = (position - object_pos).norm();
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

}  // namespace yolopnav