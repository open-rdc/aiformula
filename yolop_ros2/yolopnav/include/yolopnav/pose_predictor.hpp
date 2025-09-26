#pragma once

#include <vector>
#include <Eigen/Dense>

namespace yolopnav {

struct Position2d {
    double x = 0.0;
    double y = 0.0;
    
    Position2d() = default;
    Position2d(double x_val, double y_val) : x(x_val), y(y_val) {}
    
    Eigen::Vector2d as_array() const { return Eigen::Vector2d(x, y); }
};

struct Pose {
    Position2d pos;
    double yaw = 0.0;
    
    Pose() = default;
    Pose(const Position2d& position, double yaw_angle) : pos(position), yaw(yaw_angle) {}
};

struct VelocityState {
    double linear = 0.0;  // 線速度 [m/s]
    double angular = 0.0; // 角速度 [rad/s]
};

struct PosePredictorParams {
    // 予測パラメータ
    int horizon_length = 3;              // 予測ホライゾン長
    std::vector<double> horizon_times = {2.0, 3.0, 4.0}; // 各ホライゾンの時間[s]
    
    // 車両パラメータ
    double curvature_radius_maximum = 100.0;  // 最大曲率半径[m]
    
    // 横方向探索パラメータ
    std::vector<std::vector<double>> seek_y_positions = {
        {-1.0, -0.5, 0.0, 0.5, 1.0},     // horizon 0の横方向探索点
        {-1.0, -0.5, 0.0, 0.5, 1.0},     // horizon 1の横方向探索点
        {-1.0, -0.5, 0.0, 0.5, 1.0}      // horizon 2の横方向探索点
    };
};

// aiformulaのseek_positionsに対応する構造
using SeekPositions = std::vector<std::vector<std::vector<double>>>; // [horizon][xy][points]

class PosePredictor {
public:
    explicit PosePredictor(const PosePredictorParams& params = PosePredictorParams());
    
    // 現在速度の設定
    void setCurrentVelocity(const VelocityState& velocity);
    
    // aiformula準拠のメイン関数群
    std::vector<Eigen::Vector2d> predictRelativeEgoPositions(const std::vector<double>& curvatures);
    
    SeekPositions predictRelativeSeekPositions(const std::vector<Eigen::Vector2d>& ego_positions);
    
    SeekPositions predictAbsoluteSeekPositions(const std::vector<Eigen::Vector2d>& ego_positions,
                                             const SeekPositions& relative_seek_positions);
    
    // 単一位置予測（ユーティリティ関数）
    Pose predictPose(double curvature, double horizon_time);
    
    // パラメータ設定
    void setParams(const PosePredictorParams& params);
    PosePredictorParams getParams() const;

private:
    PosePredictorParams params_;
    VelocityState ego_current_velocity_;
    
    // 回転行列作成
    static Eigen::Matrix2d createRotationMatrix(double angle);
    
    // 位置の回転変換
    static Eigen::Vector2d rotatePosition(const Position2d& position, double angle);
};

}  // namespace yolopnav