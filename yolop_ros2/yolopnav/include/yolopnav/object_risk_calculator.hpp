#pragma once

#include <vector>
#include <Eigen/Dense>
#include "yolopnav/pose_predictor.hpp"

namespace yolopnav {

struct ObjectRiskParams {
    double risk_distance = 1.0;        // 障害物リスクが発生する距離[m]
    double risk_gain = 1.0;             // 障害物リスクのゲイン
    bool enable_object_detection = false; // 障害物検出の有効/無効
};

class ObjectRiskCalculator {
public:
    explicit ObjectRiskCalculator(const ObjectRiskParams& params = ObjectRiskParams());
    
    // aiformula準拠のメイン関数
    std::vector<std::vector<double>> computeObjectRisk(const SeekPositions& seek_positions);
    
    // 障害物位置の設定（将来的にセンサーデータから取得）
    void setDetectedObjects(const std::vector<Eigen::Vector2d>& object_positions);
    
    // パラメータ設定
    void setParams(const ObjectRiskParams& params);
    ObjectRiskParams getParams() const;

private:
    ObjectRiskParams params_;
    std::vector<Eigen::Vector2d> detected_objects_;
    
    // 単一点での障害物リスク計算
    double calculatePointRisk(const Eigen::Vector2d& position);
    
    // 障害物との距離計算
    double calculateMinDistanceToObjects(const Eigen::Vector2d& position);
};

}  // namespace yolopnav