#pragma once

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "yolopnav/pose_predictor.hpp"

namespace yolopnav {

struct LaneData {
    std::vector<Eigen::Vector3d> left_points;
    std::vector<Eigen::Vector3d> right_points;
    std::vector<Eigen::Vector3d> center_points;
};

enum class LaneSide {
    LEFT = 0,
    RIGHT = 1
};

struct RoadRiskParams {
    // aiformula準拠の道路リスクパラメータ
    double left_gradient = 5.0;            // 左車線リスクの勾配
    double right_gradient = 5.0;           // 右車線リスクの勾配
    double margin = 0.2;                   // リスクマージン
    double offset = 1.57;                  // リスクオフセット
    double gain = 5.0;                     // リスクゲイン
    
    // 利益関数パラメータ
    double benefit_scale = 1.0;            // 利益関数のスケール
    double benefit_covariance = 0.25;      // 利益関数の分散
};

struct PotentialFieldParams {
    double attractive_gain = 1.0;           // 引力ポテンシャルのゲイン
    double repulsive_gain = 1.0;            // 斥力ポテンシャルのゲイン
    double repulsive_distance = 0.5;        // 斥力が働く距離
    double grid_resolution = 0.1;           // グリッドの解像度
    double path_lookahead = 10.0;           // 経路生成の先読み距離
    int path_points = 30;                   // 生成する経路点数
    
    // ESCパラメータ
    double seek_amplitude = 0.08;           // 探索振幅
    int seek_points = 5;                    // 探索点数
    double esc_gain = -0.2;                 // ESC制御ゲイン
    std::vector<double> sin_weights = {1.0, 0.71, 0.0, -0.71, -1.0}; // 正弦波重み
};

struct PotentialFieldVisualizationParams {
    int image_width = 800;                  // 画像幅
    int image_height = 600;                 // 画像高
    double field_width = 10.0;              // フィールド幅（y: -5~5）
    double field_height = 10.0;             // フィールド高（x: 0~10）
    double min_potential = 0.0;             // 最小ポテンシャル値
    double max_potential = 10.0;            // 最大ポテンシャル値
    // スパース計算パラメータ
    int sparse_grid_x = 40;                 // x方向格子数（40点）
    int sparse_grid_y = 30;                 // y方向格子数（30点）
};

class RoadRiskCalculator {
public:
    explicit RoadRiskCalculator(const RoadRiskParams& params = RoadRiskParams());
    
    // aiformula準拠のメイン関数群
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
    calculateRoadRisk(const SeekPositions& seek_positions, const LaneData& lane_data);
    
    // 個別計算関数
    std::pair<std::vector<std::vector<double>>, double> computeRoadRisk(const SeekPositions& seek_positions, 
                                                                       const std::vector<Eigen::Vector3d>& lane_points,
                                                                       LaneSide side);
    
    std::vector<std::vector<double>> getBenefitValue(const SeekPositions& seek_positions, 
                                                   double y_hat_left, double y_hat_right);
    
    // パラメータ設定
    void setParams(const RoadRiskParams& params);
    RoadRiskParams getParams() const;

private:
    RoadRiskParams params_;
    
    // 車線推定
    double estimateYHat(const Eigen::Vector2d& position, const std::vector<Eigen::Vector3d>& lane_points);
    
    // リスク値計算
    std::vector<double> getRoadRiskValue(const std::vector<double>& seek_y_positions, 
                                       double y_hat, LaneSide side);
};

class PotentialFieldGenerator {
public:
    explicit PotentialFieldGenerator(const PotentialFieldParams& params = PotentialFieldParams());
    
    // メイン処理：座標変換後の点群からポテンシャル場による経路を生成
    nav_msgs::msg::Path generatePath(const LaneData& lane_data);
    
    
    // パラメータ設定
    void setParams(const PotentialFieldParams& params);
    PotentialFieldParams getParams() const;
    
    // 総ポテンシャル計算（可視化用にpublic）
    double calculateTotalPotential(const Eigen::Vector3d& position, const LaneData& lane_data) const;

private:
    PotentialFieldParams params_;
    
    // 両白線の中央を計算
    std::vector<Eigen::Vector3d> calculateCenterline(const std::vector<Eigen::Vector3d>& left_points,
                                                   const std::vector<Eigen::Vector3d>& right_points) const;
    
    // 引力ポテンシャル：白線中央への引力
    double calculateAttractivePotential(const Eigen::Vector3d& position, 
                                      const std::vector<Eigen::Vector3d>& centerline) const;
    
    // 斥力ポテンシャル：白線位置を避ける斥力
    double calculateRepulsivePotential(const Eigen::Vector3d& position,
                                     const std::vector<Eigen::Vector3d>& left_points,
                                     const std::vector<Eigen::Vector3d>& right_points) const;
    
    // ESCベースの勾配推定
    Eigen::Vector3d estimateGradientESC(const Eigen::Vector3d& position, const LaneData& lane_data);
    
    // 経路点列生成
    std::vector<Eigen::Vector3d> generatePathPoints(const LaneData& lane_data);
    
    // 最も近い中央線上の点を見つける
    Eigen::Vector3d findNearestCenterPoint(const Eigen::Vector3d& position,
                                         const std::vector<Eigen::Vector3d>& centerline);
    
    // 2点間距離計算
    double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const;
    
    // nav_msgs::msg::Path形式に変換
    nav_msgs::msg::Path convertToRosPath(const std::vector<Eigen::Vector3d>& path_points);
    
    // ESC用ヘルパー関数
    std::vector<Eigen::Vector3d> generateSeekPoints(const Eigen::Vector3d& center_position);
    double applyESCMovingAverage(const std::vector<double>& potential_values);
};

class PotentialFieldVisualizer {
public:
    explicit PotentialFieldVisualizer(const PotentialFieldVisualizationParams& params = PotentialFieldVisualizationParams());
    
    // スパース計算による高速ポテンシャル場可視化
    cv::Mat visualizePotentialField(const LaneData& lane_data, 
                                   const PotentialFieldGenerator& field_generator);
    
    // Pythonプロットでリアルタイム表示
    void showPotentialFieldRealtime(const LaneData& lane_data, 
                                   const PotentialFieldGenerator& field_generator);
    
    // パラメータ設定・取得
    void setParams(const PotentialFieldVisualizationParams& params);
    PotentialFieldVisualizationParams getParams() const;

private:
    PotentialFieldVisualizationParams params_;
    
    // ワールド座標をピクセル座標に変換
    cv::Point2i worldToPixel(double x, double y) const;
    
    // ピクセル座標をワールド座標に変換
    std::pair<double, double> pixelToWorld(int px, int py) const;
    
    // ポテンシャル値をカラーマップに変換
    cv::Vec3b potentialToColor(double potential) const;
    
    // ポテンシャル値を正規化
    double normalizePotential(double potential) const;
    
    // スパース計算ヘルパー関数
    std::vector<Eigen::Vector3d> calculateCenterlineFromLaneData(const LaneData& lane_data) const;
    double calculateOptimizedPotential(const Eigen::Vector3d& position, 
                                     const LaneData& lane_data,
                                     const std::vector<Eigen::Vector3d>& centerline) const;
    void drawSimpleGrid(cv::Mat& image) const;
    void addLabelsToImage(cv::Mat& image) const;
    
    // スパースグリッド関連
    std::vector<cv::Point2i> generateSparseGridPoints() const;
    cv::Mat interpolateSparseToFull(const std::vector<cv::Point2i>& grid_points,
                                   const std::vector<double>& potential_values) const;
};

}  // namespace yolopnav