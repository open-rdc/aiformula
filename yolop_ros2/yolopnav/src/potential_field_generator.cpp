#include "yolopnav/potential_field_generator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace yolopnav {

// RoadRiskCalculator implementation
RoadRiskCalculator::RoadRiskCalculator(const RoadRiskParams& params) : params_(params) {}

void RoadRiskCalculator::setParams(const RoadRiskParams& params) {
    params_ = params;
}

RoadRiskParams RoadRiskCalculator::getParams() const {
    return params_;
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>>
RoadRiskCalculator::calculateRoadRisk(const SeekPositions& seek_positions, const LaneData& lane_data) {
    // 左車線リスク計算
    auto [left_road_risk, left_y_hat] = computeRoadRisk(seek_positions, lane_data.left_points, LaneSide::LEFT);
    
    // 右車線リスク計算  
    auto [right_road_risk, right_y_hat] = computeRoadRisk(seek_positions, lane_data.right_points, LaneSide::RIGHT);
    
    // 利益関数計算
    auto benefit = getBenefitValue(seek_positions, left_y_hat, right_y_hat);
    
    return std::make_tuple(left_road_risk, right_road_risk, benefit);
}

std::pair<std::vector<std::vector<double>>, double> RoadRiskCalculator::computeRoadRisk(
    const SeekPositions& seek_positions, 
    const std::vector<Eigen::Vector3d>& lane_points,
    LaneSide side) {
    
    std::vector<std::vector<double>> risks;
    double y_hat = 0.0;
    
    for (const auto& horizon_positions : seek_positions) {
        std::vector<double> horizon_risks;
        
        // 中央の探索点でy_hatを推定
        if (!horizon_positions[0].empty()) {
            size_t center_idx = horizon_positions[0].size() / 2;
            Eigen::Vector2d center_point(horizon_positions[0][center_idx], 
                                       horizon_positions[1][center_idx]);
            y_hat = estimateYHat(center_point, lane_points);
        }
        
        // 各探索点でのリスクを計算
        horizon_risks = getRoadRiskValue(horizon_positions[1], y_hat, side);
        risks.push_back(horizon_risks);
    }
    
    return std::make_pair(risks, y_hat);
}

std::vector<std::vector<double>> RoadRiskCalculator::getBenefitValue(
    const SeekPositions& seek_positions, 
    double y_hat_left, double y_hat_right) {
    
    std::vector<std::vector<double>> benefits;
    double y_hat_center = (y_hat_left + y_hat_right) * 0.5;  // aiformula準拠
    
    for (const auto& horizon_positions : seek_positions) {
        std::vector<double> horizon_benefits;
        
        // 各探索点での利益を計算（ガウシアン分布）
        for (double seek_y : horizon_positions[1]) {
            double sigma_squared = params_.benefit_covariance;
            double distance_to_center = seek_y - y_hat_center;
            double benefit = params_.benefit_scale * 
                           std::exp(-0.5 * distance_to_center * distance_to_center / sigma_squared);
            horizon_benefits.push_back(benefit);
        }
        
        benefits.push_back(horizon_benefits);
    }
    
    return benefits;
}

double RoadRiskCalculator::estimateYHat(const Eigen::Vector2d& position, 
                                      const std::vector<Eigen::Vector3d>& lane_points) {
    if (lane_points.empty()) {
        return 0.0;
    }
    
    // 最も近い車線点のy座標を返す（簡単な実装）
    double min_distance = std::numeric_limits<double>::max();
    double y_hat = 0.0;
    
    for (const auto& point : lane_points) {
        Eigen::Vector2d lane_2d(point.x(), point.y());
        double distance = (position - lane_2d).norm();
        if (distance < min_distance) {
            min_distance = distance;
            y_hat = point.y();
        }
    }
    
    return y_hat;
}

std::vector<double> RoadRiskCalculator::getRoadRiskValue(
    const std::vector<double>& seek_y_positions, 
    double y_hat, 
    LaneSide side) {
    
    std::vector<double> risks;
    
    for (double seek_y : seek_y_positions) {
        double sigma = seek_y - y_hat;
        double risk;
        
        // aiformula準拠のリスク計算
        if (side == LaneSide::RIGHT) {
            risk = params_.gain * 
                  (-std::atan(params_.left_gradient * (sigma + params_.margin)) + params_.offset);
        } else { // LaneSide::LEFT
            risk = params_.gain * 
                  (std::atan(params_.right_gradient * (sigma + params_.margin)) + params_.offset);
        }
        
        risks.push_back(risk);
    }
    
    return risks;
}

// PotentialFieldGenerator implementation
PotentialFieldGenerator::PotentialFieldGenerator(const PotentialFieldParams& params)
    : params_(params) {}

void PotentialFieldGenerator::setParams(const PotentialFieldParams& params) {
    params_ = params;
}

PotentialFieldParams PotentialFieldGenerator::getParams() const {
    return params_;
}

nav_msgs::msg::Path PotentialFieldGenerator::generatePath(const LaneData& lane_data) {
    if (lane_data.left_points.empty() || lane_data.right_points.empty()) {
        return nav_msgs::msg::Path();  // 空のパスを返す
    }

    // ポテンシャル場を基に経路点を生成
    auto path_points = generatePathPoints(lane_data);
    
    // nav_msgs::msg::Path形式に変換
    return convertToRosPath(path_points);
}




std::vector<Eigen::Vector3d> PotentialFieldGenerator::calculateCenterline(
    const std::vector<Eigen::Vector3d>& left_points,
    const std::vector<Eigen::Vector3d>& right_points) const {
    
    std::vector<Eigen::Vector3d> centerline;
    
    // 両方の白線点群の最小サイズに合わせる
    size_t min_size = std::min(left_points.size(), right_points.size());
    
    for (size_t i = 0; i < min_size; ++i) {
        // 左右の白線の中点を計算
        Eigen::Vector3d center_point = (left_points[i] + right_points[i]) * 0.5;
        centerline.push_back(center_point);
    }
    
    return centerline;
}

double PotentialFieldGenerator::calculateAttractivePotential(
    const Eigen::Vector3d& position, 
    const std::vector<Eigen::Vector3d>& centerline) const {
    
    if (centerline.empty()) {
        return 0.0;
    }
    
    // 最も近い中央線上の点を見つける
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& center_point : centerline) {
        double distance = calculateDistance(position, center_point);
        min_distance = std::min(min_distance, distance);
    }
    
    // 二次ポテンシャル（距離の二乗に比例）
    return 0.5 * params_.attractive_gain * min_distance * min_distance;
}

double PotentialFieldGenerator::calculateRepulsivePotential(
    const Eigen::Vector3d& position,
    const std::vector<Eigen::Vector3d>& left_points,
    const std::vector<Eigen::Vector3d>& right_points) const {
    
    double total_repulsive = 0.0;
    
    // 左白線からの斥力
    for (const auto& left_point : left_points) {
        double distance = calculateDistance(position, left_point);
        if (distance <= params_.repulsive_distance && distance > 0.0) {
            double repulsive = 0.5 * params_.repulsive_gain * 
                             std::pow((1.0 / distance - 1.0 / params_.repulsive_distance), 2);
            total_repulsive += repulsive;
        }
    }
    
    // 右白線からの斥力
    for (const auto& right_point : right_points) {
        double distance = calculateDistance(position, right_point);
        if (distance <= params_.repulsive_distance && distance > 0.0) {
            double repulsive = 0.5 * params_.repulsive_gain * 
                             std::pow((1.0 / distance - 1.0 / params_.repulsive_distance), 2);
            total_repulsive += repulsive;
        }
    }
    
    return total_repulsive;
}

double PotentialFieldGenerator::calculateTotalPotential(
    const Eigen::Vector3d& position, 
    const LaneData& lane_data) const {
    
    // 中央線を計算
    auto centerline = calculateCenterline(lane_data.left_points, lane_data.right_points);
    
    // 引力ポテンシャル（中央線への引力）
    double attractive = calculateAttractivePotential(position, centerline);
    
    // 斥力ポテンシャル（白線からの斥力）
    double repulsive = calculateRepulsivePotential(position, lane_data.left_points, lane_data.right_points);
    
    return attractive + repulsive;
}

Eigen::Vector3d PotentialFieldGenerator::estimateGradientESC(
    const Eigen::Vector3d& position, 
    const LaneData& lane_data) {
    
    Eigen::Vector3d gradient(0.0, 0.0, 0.0);
    
    // X方向のESC勾配推定
    std::vector<Eigen::Vector3d> x_seek_points = generateSeekPoints(position);
    std::vector<double> x_potentials;
    for (const auto& seek_point : x_seek_points) {
        Eigen::Vector3d x_point = position;
        x_point.x() = seek_point.x();
        x_potentials.push_back(calculateTotalPotential(x_point, lane_data));
    }
    gradient.x() = applyESCMovingAverage(x_potentials);
    
    // Y方向のESC勾配推定
    std::vector<double> y_potentials;
    for (const auto& seek_point : x_seek_points) {
        Eigen::Vector3d y_point = position;
        y_point.y() = seek_point.y();
        y_potentials.push_back(calculateTotalPotential(y_point, lane_data));
    }
    gradient.y() = applyESCMovingAverage(y_potentials);
    
    // z方向は0に固定（2D平面での移動を想定）
    gradient.z() = 0.0;
    
    return gradient;
}

std::vector<Eigen::Vector3d> PotentialFieldGenerator::generatePathPoints(const LaneData& lane_data) {
    
    std::vector<Eigen::Vector3d> path_points;
    // ロボット座標系では常に原点(0,0,0)から開始
    Eigen::Vector3d current_position(0.0, 0.0, 0.0);
    
    for (int i = 0; i < params_.path_points; ++i) {
        path_points.push_back(current_position);
        
        // ESCベースの勾配推定（負の勾配方向に移動）
        Eigen::Vector3d gradient = estimateGradientESC(current_position, lane_data);
        
        // 勾配の正規化
        double gradient_norm = gradient.norm();
        if (gradient_norm > 0.0) {
            gradient = gradient / gradient_norm;
        }
        
        // 次の点を計算（負の勾配方向に一定距離移動）
        double step_size = params_.path_lookahead / params_.path_points;
        current_position = current_position - gradient * step_size;
    }
    
    return path_points;
}

Eigen::Vector3d PotentialFieldGenerator::findNearestCenterPoint(
    const Eigen::Vector3d& position,
    const std::vector<Eigen::Vector3d>& centerline) {
    
    if (centerline.empty()) {
        return position;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d nearest_point = centerline[0];
    
    for (const auto& center_point : centerline) {
        double distance = calculateDistance(position, center_point);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = center_point;
        }
    }
    
    return nearest_point;
}

double PotentialFieldGenerator::calculateDistance(
    const Eigen::Vector3d& p1, 
    const Eigen::Vector3d& p2) const {
    
    return (p1 - p2).norm();
}

nav_msgs::msg::Path PotentialFieldGenerator::convertToRosPath(
    const std::vector<Eigen::Vector3d>& path_points) {
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "base_link";
    
    for (const auto& point : path_points) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        
        // 姿勢は単位クォータニオンに設定
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose_stamped);
    }
    
    return path_msg;
}

std::vector<Eigen::Vector3d> PotentialFieldGenerator::generateSeekPoints(
    const Eigen::Vector3d& center_position) {
    
    std::vector<Eigen::Vector3d> seek_points;
    
    // ESC探索点を生成（正弦波ベースで5点）
    for (int i = 0; i < params_.seek_points; ++i) {
        double offset = params_.sin_weights[i] * params_.seek_amplitude;
        
        // X方向とY方向に同じオフセットを適用
        Eigen::Vector3d seek_point = center_position;
        seek_point.x() += offset;
        seek_point.y() += offset;
        
        seek_points.push_back(seek_point);
    }
    
    return seek_points;
}

double PotentialFieldGenerator::applyESCMovingAverage(
    const std::vector<double>& potential_values) {
    
    if (potential_values.size() != static_cast<size_t>(params_.seek_points) || params_.seek_points != 5) {
        return 0.0;  // エラー処理
    }
    
    // 中央点（index=2）を基準に他の点との差分を計算
    int center_idx = 2;
    std::vector<double> differences;
    
    for (int i = 0; i < params_.seek_points; ++i) {
        if (i != center_idx) {
            differences.push_back(potential_values[i] - potential_values[center_idx]);
        }
    }
    
    // 正弦波重み付き移動平均でリスク勾配を近似
    double weighted_sum = 0.0;
    int diff_idx = 0;
    
    for (int i = 0; i < params_.seek_points; ++i) {
        if (i != center_idx) {
            weighted_sum += params_.sin_weights[i] * differences[diff_idx];
            diff_idx++;
        }
    }
    
    return params_.esc_gain * weighted_sum;
}

// PotentialFieldVisualizer implementation
PotentialFieldVisualizer::PotentialFieldVisualizer(const PotentialFieldVisualizationParams& params)
    : params_(params) {}

void PotentialFieldVisualizer::setParams(const PotentialFieldVisualizationParams& params) {
    params_ = params;
}

PotentialFieldVisualizationParams PotentialFieldVisualizer::getParams() const {
    return params_;
}

cv::Mat PotentialFieldVisualizer::visualizePotentialField(const LaneData& lane_data, 
                                                        const PotentialFieldGenerator& field_generator) {
    cv::Mat heatmap(params_.image_height, params_.image_width, CV_8UC3);
    
    // スパースグリッド点を生成（40x30 = 1200点のみ計算）
    auto sparse_points = generateSparseGridPoints();
    std::vector<double> potential_values;
    potential_values.reserve(sparse_points.size());
    
    // 中央線を事前計算
    auto centerline = calculateCenterlineFromLaneData(lane_data);
    
    // スパースグリッド点のみでポテンシャル計算
    for (const auto& pixel_point : sparse_points) {
        auto [world_x, world_y] = pixelToWorld(pixel_point.x, pixel_point.y);
        Eigen::Vector3d position(world_x, world_y, 0.0);
        double potential = calculateOptimizedPotential(position, lane_data, centerline);
        potential_values.push_back(potential);
    }
    
    // スパースグリッドからフル解像度に補間
    heatmap = interpolateSparseToFull(sparse_points, potential_values);
    
    // 白線を描画
    for (const auto& left_point : lane_data.left_points) {
        cv::Point2i pixel = worldToPixel(left_point.x(), left_point.y());
        if (pixel.x >= 0 && pixel.x < params_.image_width && 
            pixel.y >= 0 && pixel.y < params_.image_height) {
            cv::circle(heatmap, pixel, 3, cv::Scalar(255, 255, 255), -1);
        }
    }
    
    for (const auto& right_point : lane_data.right_points) {
        cv::Point2i pixel = worldToPixel(right_point.x(), right_point.y());
        if (pixel.x >= 0 && pixel.x < params_.image_width && 
            pixel.y >= 0 && pixel.y < params_.image_height) {
            cv::circle(heatmap, pixel, 3, cv::Scalar(255, 255, 255), -1);
        }
    }
    
    // ロボット位置（原点）を描画
    cv::Point2i robot_pixel = worldToPixel(0.0, 0.0);
    if (robot_pixel.x >= 0 && robot_pixel.x < params_.image_width && 
        robot_pixel.y >= 0 && robot_pixel.y < params_.image_height) {
        cv::circle(heatmap, robot_pixel, 5, cv::Scalar(0, 255, 0), -1);
    }
    
    return heatmap;
}

void PotentialFieldVisualizer::showPotentialFieldRealtime(const LaneData& lane_data, 
                                                        const PotentialFieldGenerator& field_generator) {
    // スパース計算による高速ヒートマップ生成
    cv::Mat heatmap = visualizePotentialField(lane_data, field_generator);
    
    // 軸ラベルと格子を追加
    cv::Mat display_image = heatmap.clone();
    
    // 簡易格子線描画（計算量削減）
    drawSimpleGrid(display_image);
    
    // ラベル描画（テキスト数削減）
    addLabelsToImage(display_image);
    
    // OpenCVウィンドウで表示
    cv::imshow("Potential Field Visualization", display_image);
    cv::waitKey(1);  // 1ms待機でリアルタイム更新
}

cv::Point2i PotentialFieldVisualizer::worldToPixel(double x, double y) const {
    // ロボット座標系: x軸縦（前方: 0~10m）、y軸横（左右: -5~5m）
    // 画像座標系: x軸横（右方）、y軸縦（下方）
    // 座標変換: ロボットのx軸を画像のy軸に、ロボットのy軸を画像のx軸に対応
    
    // ワールド座標を画像座標に変換
    // y座標: -5~5 -> 0~image_width
    int px = static_cast<int>((y + 5.0) * params_.image_width / 10.0);
    // x座標: 0~10 -> image_height~0 (上下反転)
    int py = static_cast<int>((10.0 - x) * params_.image_height / 10.0);
    
    return cv::Point2i(px, py);
}

std::pair<double, double> PotentialFieldVisualizer::pixelToWorld(int px, int py) const {
    // 画像座標をワールド座標に変換
    // px: 0~image_width -> y: -5~5
    double y = (static_cast<double>(px) * 10.0 / params_.image_width) - 5.0;
    // py: 0~image_height -> x: 10~0
    double x = 10.0 - (static_cast<double>(py) * 10.0 / params_.image_height);
    
    return std::make_pair(x, y);
}

cv::Vec3b PotentialFieldVisualizer::potentialToColor(double potential) const {
    // ポテンシャル値を正規化
    double normalized = normalizePotential(potential);
    
    // ヒートマップカラー（青→緑→黄→赤）
    cv::Vec3b color;
    
    if (normalized < 0.25) {
        // 青 → シアン (BGR形式)
        double ratio = normalized / 0.25;
        color[0] = 255;                                     // B (青固定)
        color[1] = static_cast<uchar>(255 * ratio);         // G (緑増加)
        color[2] = 0;                                       // R (赤なし)
    } else if (normalized < 0.5) {
        // シアン → 緑
        double ratio = (normalized - 0.25) / 0.25;
        color[0] = static_cast<uchar>(255 * (1.0 - ratio)); // B (青減少)
        color[1] = 255;                                     // G (緑固定)
        color[2] = 0;                                       // R (赤なし)
    } else if (normalized < 0.75) {
        // 緑 → 黄
        double ratio = (normalized - 0.5) / 0.25;
        color[0] = 0;                                       // B (青なし)
        color[1] = 255;                                     // G (緑固定)
        color[2] = static_cast<uchar>(255 * ratio);         // R (赤増加)
    } else {
        // 黄 → 赤
        double ratio = (normalized - 0.75) / 0.25;
        color[0] = 0;                                       // B (青なし)
        color[1] = static_cast<uchar>(255 * (1.0 - ratio)); // G (緑減少)
        color[2] = 255;                                     // R (赤固定)
    }
    
    return color;
}

double PotentialFieldVisualizer::normalizePotential(double potential) const {
    // ポテンシャル値を[0, 1]の範囲に正規化
    double normalized = (potential - params_.min_potential) / (params_.max_potential - params_.min_potential);
    return std::clamp(normalized, 0.0, 1.0);
}

// PotentialFieldVisualizerの最適化ヘルパー関数
std::vector<Eigen::Vector3d> PotentialFieldVisualizer::calculateCenterlineFromLaneData(const LaneData& lane_data) const {
    std::vector<Eigen::Vector3d> centerline;
    size_t min_size = std::min(lane_data.left_points.size(), lane_data.right_points.size());
    
    centerline.reserve(min_size);  // メモリ確保を最適化
    for (size_t i = 0; i < min_size; ++i) {
        centerline.emplace_back((lane_data.left_points[i] + lane_data.right_points[i]) * 0.5);
    }
    
    return centerline;
}

double PotentialFieldVisualizer::calculateOptimizedPotential(
    const Eigen::Vector3d& position, 
    const LaneData& lane_data,
    const std::vector<Eigen::Vector3d>& centerline) const {
    
    // 引力ポテンシャル計算（事前計算された中央線を使用）
    double min_center_distance = std::numeric_limits<double>::max();
    for (const auto& center_point : centerline) {
        double distance = (position - center_point).norm();
        min_center_distance = std::min(min_center_distance, distance);
    }
    double attractive = 0.5 * 1.0 * min_center_distance * min_center_distance;  // attractive_gain = 1.0固定
    
    // 斥力ポテンシャル計算（距離閾値で早期終了）
    double repulsive = 0.0;
    const double repulsive_distance = 0.5;  // params_.repulsive_distance固定
    const double repulsive_gain = 1.0;      // params_.repulsive_gain固定
    
    // 左白線からの斥力（距離閾値チェックを先に実行）
    for (const auto& left_point : lane_data.left_points) {
        double distance = (position - left_point).norm();
        if (distance <= repulsive_distance && distance > 1e-6) {  // ゼロ除算回避
            double repulsive_term = 0.5 * repulsive_gain * 
                                   std::pow((1.0 / distance - 1.0 / repulsive_distance), 2);
            repulsive += repulsive_term;
        }
    }
    
    // 右白線からの斥力
    for (const auto& right_point : lane_data.right_points) {
        double distance = (position - right_point).norm();
        if (distance <= repulsive_distance && distance > 1e-6) {  // ゼロ除算回避
            double repulsive_term = 0.5 * repulsive_gain * 
                                   std::pow((1.0 / distance - 1.0 / repulsive_distance), 2);
            repulsive += repulsive_term;
        }
    }
    
    return attractive + repulsive;
}

void PotentialFieldVisualizer::drawSimpleGrid(cv::Mat& image) const {
    const cv::Scalar grid_color(128, 128, 128);
    const int grid_step_x = params_.image_width / 10;
    const int grid_step_y = params_.image_height / 10;
    
    // 縦線と横線を一度に描画
    for (int i = 1; i < 10; ++i) {
        // 縦線
        cv::line(image, cv::Point(i * grid_step_x, 0), 
                cv::Point(i * grid_step_x, params_.image_height), grid_color, 1);
        // 横線
        cv::line(image, cv::Point(0, i * grid_step_y), 
                cv::Point(params_.image_width, i * grid_step_y), grid_color, 1);
    }
}

void PotentialFieldVisualizer::addLabelsToImage(cv::Mat& image) const {
    const cv::Scalar white(255, 255, 255);
    
    // 基本情報のみ表示（テキスト描画回数削減）
    cv::putText(image, "Potential Field", cv::Point(10, 25), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 2);
    cv::putText(image, "x:0-10m y:-5-5m", cv::Point(10, 50), 
               cv::FONT_HERSHEY_SIMPLEX, 0.4, white, 1);
    cv::putText(image, "Blue:Low Red:High", cv::Point(params_.image_width - 150, params_.image_height - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.4, white, 1);
}

std::vector<cv::Point2i> PotentialFieldVisualizer::generateSparseGridPoints() const {
    std::vector<cv::Point2i> grid_points;
    grid_points.reserve(params_.sparse_grid_x * params_.sparse_grid_y);
    
    // スパースグリッドの間隔を計算
    int step_x = params_.image_width / params_.sparse_grid_x;
    int step_y = params_.image_height / params_.sparse_grid_y;
    
    // 格子点を生成（端っこは含めない）
    for (int gy = 0; gy < params_.sparse_grid_y; ++gy) {
        for (int gx = 0; gx < params_.sparse_grid_x; ++gx) {
            int px = gx * step_x + step_x / 2;  // グリッドセルの中央
            int py = gy * step_y + step_y / 2;
            
            // 画像範囲内のチェック
            if (px >= 0 && px < params_.image_width && py >= 0 && py < params_.image_height) {
                grid_points.emplace_back(px, py);
            }
        }
    }
    
    return grid_points;
}

cv::Mat PotentialFieldVisualizer::interpolateSparseToFull(
    const std::vector<cv::Point2i>& grid_points,
    const std::vector<double>& potential_values) const {
    
    cv::Mat heatmap(params_.image_height, params_.image_width, CV_8UC3);
    
    if (grid_points.size() != potential_values.size()) {
        return heatmap;  // サイズ不一致エラー
    }
    
    // グリッド間隔
    int step_x = params_.image_width / params_.sparse_grid_x;
    int step_y = params_.image_height / params_.sparse_grid_y;
    
    // 各グリッドセルを均一に塗りつぶし（簡単な補間）
    for (size_t i = 0; i < grid_points.size(); ++i) {
        const cv::Point2i& center = grid_points[i];
        double potential = potential_values[i];
        cv::Vec3b color = potentialToColor(potential);
        
        // グリッドセルの範囲を計算
        int start_x = std::max(0, center.x - step_x / 2);
        int end_x = std::min(params_.image_width, center.x + step_x / 2);
        int start_y = std::max(0, center.y - step_y / 2);
        int end_y = std::min(params_.image_height, center.y + step_y / 2);
        
        // セル内を同じ色で塗りつぶし
        for (int py = start_y; py < end_y; ++py) {
            for (int px = start_x; px < end_x; ++px) {
                heatmap.at<cv::Vec3b>(py, px) = color;
            }
        }
    }
    
    return heatmap;
}

}  // namespace yolopnav