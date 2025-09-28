#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <chrono>

namespace yolopnav {

struct LaneObservation {
    std::vector<double> y_positions;      // y座標 [m]
    std::vector<double> x_positions;      // 対応する横位置 [m]
    std::vector<double> variances;        // 各観測の分散
    std::chrono::steady_clock::time_point timestamp;
    double confidence;                    // 観測の信頼度 [0-1]
};

struct KalmanFilterState {
    Eigen::VectorXd state;               // 状態ベクトル [20次元: 位置10 + 速度10]
    Eigen::MatrixXd covariance;          // 共分散行列 [20x20]
    std::chrono::steady_clock::time_point last_update;
    bool is_initialized;
    int track_id;
    double confidence;
};

class LaneKalmanFilter {
public:
    static constexpr int NUM_SAMPLES = 10;        // y=1m～10mでサンプリング
    static constexpr int STATE_DIM = 20;          // 位置10 + 速度10
    static constexpr double SAMPLE_INTERVAL = 1.0; // 1m間隔
    static constexpr double MIN_Y = 1.0;          // 最小y距離
    static constexpr double MAX_Y = 10.0;         // 最大y距離

    // 観測信頼性検証パラメータ
    static constexpr double MAX_EXPECTED_CHANGE = 0.1;  // 最大許容変化量 [m]
    static constexpr double RELIABILITY_DECAY_FACTOR = 10.0;  // 信頼性減衰係数

    explicit LaneKalmanFilter(int track_id = 0);
    ~LaneKalmanFilter() = default;

    // カルマンフィルタの基本操作
    void predict(double dt);
    void update(const LaneObservation& observation);

    // 初期化・リセット
    void initialize(const LaneObservation& initial_observation);
    void reset();
    bool isInitialized() const { return state_.is_initialized; }

    // 状態取得
    std::vector<Eigen::Vector3d> getUniformPoints(double interval = 0.5) const;
    std::vector<double> getLateralPositions() const;
    double getConfidence() const { return state_.confidence; }
    int getTrackId() const { return state_.track_id; }

    // 幾何的制約チェック
    bool checkGeometricConstraints(const LaneKalmanFilter& other_lane,
                                 double min_width = 0.3,
                                 double max_width = 0.5) const;

    // 観測関連付け（データアソシエーション）
    double calculateMahalanobisDistance(const LaneObservation& observation) const;
    bool isValidAssociation(const LaneObservation& observation,
                          double max_mahalanobis = 9.0) const;

private:
    KalmanFilterState state_;
    Eigen::MatrixXd F_;                  // 状態遷移行列
    Eigen::MatrixXd H_;                  // 観測行列
    Eigen::MatrixXd Q_;                  // プロセスノイズ共分散
    Eigen::MatrixXd R_;                  // 観測ノイズ共分散
    Eigen::MatrixXd I_;                  // 単位行列

    // 初期化とパラメータ設定
    void initializeMatrices();
    void setProcessNoise(double dt);
    void setObservationNoise(const LaneObservation& observation);

    // 観測処理
    std::pair<Eigen::VectorXd, Eigen::MatrixXd>
    createObservationVector(const LaneObservation& observation) const;

    // ユーティリティ
    double interpolatePosition(double y) const;
    double interpolateFromObservation(double y, const LaneObservation& observation) const;
    std::vector<double> getVelocities() const;
    void updateConfidence(const LaneObservation& observation);

    // 観測信頼性検証
    double calculateObservationReliability(const LaneObservation& observation) const;

    // パラメータ（チューニング用）
    double base_process_noise_;          // 基本プロセスノイズ
    double velocity_process_noise_;      // 速度プロセスノイズ
    double base_observation_noise_;      // 基本観測ノイズ
    double distance_noise_factor_;       // 距離に依存するノイズ係数
    double confidence_decay_rate_;       // 信頼度減衰率
};

class LaneKalmanManager {
public:
    LaneKalmanManager();
    ~LaneKalmanManager() = default;

    // レーン管理
    void updateLanes(const std::vector<LaneObservation>& observations);
    std::vector<std::vector<Eigen::Vector3d>> getFilteredLanes(double interval = 0.5) const;

    // トラック管理
    void predictAll(double dt);
    void pruneOldTracks(double max_age_seconds = 2.0);

    // 結果取得
    bool hasLeftLane() const;
    bool hasRightLane() const;
    std::vector<Eigen::Vector3d> getLeftLanePoints(double interval = 0.5) const;
    std::vector<Eigen::Vector3d> getRightLanePoints(double interval = 0.5) const;
    std::vector<Eigen::Vector3d> getCenterLanePoints(double interval = 0.5) const;

private:
    std::vector<std::unique_ptr<LaneKalmanFilter>> tracks_;
    int next_track_id_;
    std::chrono::steady_clock::time_point last_update_;

    // データアソシエーション
    std::vector<std::pair<int, int>> associateObservations(
        const std::vector<LaneObservation>& observations);

    // レーン分類（左右判定）
    enum class LaneType { LEFT, RIGHT, CENTER, UNKNOWN };
    LaneType classifyLane(const LaneKalmanFilter& track) const;

    // 幾何的制約による検証
    void validateLanePairs();
};

}  // namespace yolopnav