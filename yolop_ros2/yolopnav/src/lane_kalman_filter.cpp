#include "yolopnav/lane_kalman_filter.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace yolopnav {

LaneKalmanFilter::LaneKalmanFilter(int track_id)
    : base_process_noise_(5e-2),
      velocity_process_noise_(5e-2),
      base_observation_noise_(0.03),
      distance_noise_factor_(0.02),
      confidence_decay_rate_(0.95) {

    state_.track_id = track_id;
    state_.is_initialized = false;
    state_.confidence = 0.0;

    initializeMatrices();
}

void LaneKalmanFilter::initializeMatrices() {
    // 状態ベクトル: [x1, x2, ..., x10, vx1, vx2, ..., vx10]
    state_.state = Eigen::VectorXd::Zero(STATE_DIM);
    state_.covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1.0;

    // 状態遷移行列 F (定常速度モデル)
    F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    // F の右上ブロックに dt を設定（predict時に更新）

    // 観測行列 H (位置のみ観測)
    H_ = Eigen::MatrixXd::Zero(NUM_SAMPLES, STATE_DIM);
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        H_(i, i) = 1.0;  // 位置成分のみ観測
    }

    // 単位行列
    I_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    // プロセスノイズ Q と観測ノイズ R は動的に設定
}

void LaneKalmanFilter::initialize(const LaneObservation& initial_observation) {
    if (initial_observation.y_positions.size() != initial_observation.x_positions.size()) {
        return;
    }

    // 初期状態を観測から設定
    state_.state.setZero();

    // 位置の初期化（補間で全サンプル点を設定）
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double y = MIN_Y + i * SAMPLE_INTERVAL;
        double x = interpolateFromObservation(y, initial_observation);
        state_.state[i] = x;  // 位置
        state_.state[i + NUM_SAMPLES] = 0.0;  // 速度は0で初期化
    }

    // 初期共分散を設定
    state_.covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double y = MIN_Y + i * SAMPLE_INTERVAL;
        double variance = base_observation_noise_ + distance_noise_factor_ * y;
        state_.covariance(i, i) = variance * variance;  // 位置の不確実性
        state_.covariance(i + NUM_SAMPLES, i + NUM_SAMPLES) = velocity_process_noise_;  // 速度の不確実性
    }

    state_.is_initialized = true;
    state_.confidence = initial_observation.confidence;
    state_.last_update = initial_observation.timestamp;
}

void LaneKalmanFilter::predict(double dt) {
    if (!state_.is_initialized) {
        return;
    }

    // 状態遷移行列 F を更新（dt を設定）
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        F_(i, i + NUM_SAMPLES) = dt;  // 位置 += 速度 * dt
    }

    setProcessNoise(dt);

    // 予測ステップ
    state_.state = F_ * state_.state;
    state_.covariance = F_ * state_.covariance * F_.transpose() + Q_;

    // 信頼度の減衰
    state_.confidence *= confidence_decay_rate_;
}

void LaneKalmanFilter::update(const LaneObservation& observation) {
    if (!state_.is_initialized) {
        initialize(observation);
        return;
    }

    auto [z, R] = createObservationVector(observation);
    if (z.size() == 0) {
        return;  // 有効な観測がない
    }

    // 観測に対応する H 行列を作成
    int obs_dim = z.size();
    Eigen::MatrixXd H_obs = Eigen::MatrixXd::Zero(obs_dim, STATE_DIM);

    int obs_idx = 0;
    for (int i = 0; i < NUM_SAMPLES && obs_idx < obs_dim; ++i) {
        double y = MIN_Y + i * SAMPLE_INTERVAL;

        // 観測にこのy距離の情報があるかチェック
        bool has_observation = false;
        for (size_t j = 0; j < observation.y_positions.size(); ++j) {
            if (std::abs(observation.y_positions[j] - y) < SAMPLE_INTERVAL * 0.5) {
                has_observation = true;
                break;
            }
        }

        if (has_observation) {
            H_obs(obs_idx, i) = 1.0;  // この位置を観測
            obs_idx++;
        }
    }

    // カルマンゲイン計算
    Eigen::MatrixXd S = H_obs * state_.covariance * H_obs.transpose() + R;
    Eigen::MatrixXd K = state_.covariance * H_obs.transpose() * S.inverse();

    // 更新ステップ
    Eigen::VectorXd y_residual = z - H_obs * state_.state;
    state_.state = state_.state + K * y_residual;
    state_.covariance = (I_ - K * H_obs) * state_.covariance;

    // 信頼度更新
    updateConfidence(observation);
    state_.last_update = observation.timestamp;
}

void LaneKalmanFilter::setProcessNoise(double dt) {
    Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);

    // 位置のプロセスノイズ
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        Q_(i, i) = base_process_noise_ * dt * dt;
    }

    // 速度のプロセスノイズ
    for (int i = NUM_SAMPLES; i < STATE_DIM; ++i) {
        Q_(i, i) = velocity_process_noise_ * dt;
    }
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd>
LaneKalmanFilter::createObservationVector(const LaneObservation& observation) const {
    std::vector<double> valid_observations;
    std::vector<double> valid_variances;

    // 観測信頼性を計算
    double reliability = calculateObservationReliability(observation);

    // 各サンプル点での観測値を収集
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double y = MIN_Y + i * SAMPLE_INTERVAL;

        // この y 距離での観測があるかチェック
        for (size_t j = 0; j < observation.y_positions.size(); ++j) {
            if (std::abs(observation.y_positions[j] - y) < SAMPLE_INTERVAL * 0.5) {
                valid_observations.push_back(observation.x_positions[j]);

                // 観測ノイズを設定（距離依存 + 信頼性調整）
                double base_variance = observation.variances.size() > j ?
                    observation.variances[j] :
                    std::pow(base_observation_noise_ + distance_noise_factor_ * y, 2);

                // 信頼性が低い場合は観測ノイズを大きくする
                double reliability_factor = 1.0 / std::max(0.01, reliability);
                double adjusted_variance = base_variance * reliability_factor;

                valid_variances.push_back(adjusted_variance);
                break;
            }
        }
    }

    // 観測ベクトルと共分散行列を構築
    int obs_dim = valid_observations.size();
    Eigen::VectorXd z(obs_dim);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(obs_dim, obs_dim);

    for (int i = 0; i < obs_dim; ++i) {
        z[i] = valid_observations[i];
        R(i, i) = valid_variances[i];
    }

    return {z, R};
}

double LaneKalmanFilter::interpolateFromObservation(double y, const LaneObservation& observation) const {
    if (observation.y_positions.empty()) {
        return 0.0;
    }

    // 最も近い観測点を見つけて線形補間
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < observation.y_positions.size(); ++i) {
        double dist = std::abs(observation.y_positions[i] - y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 単純な最近傍補間（後で線形補間に拡張可能）
    return observation.x_positions[closest_idx];
}

double LaneKalmanFilter::interpolatePosition(double y) const {
    if (!state_.is_initialized) {
        return 0.0;
    }

    // 範囲外の場合は線形外挿で連続性を保つ
    if (y < MIN_Y) {
        // 前方外挿：最初の2点（y=1, y=2）を使用
        if (NUM_SAMPLES >= 2) {
            double y1 = MIN_Y;
            double y2 = MIN_Y + SAMPLE_INTERVAL;
            double x1 = state_.state[0];
            double x2 = state_.state[1];

            // 線形外挿: x = x1 + (x2-x1) * (y-y1)/(y2-y1)
            return x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        } else {
            return state_.state[0];
        }
    }

    if (y > MAX_Y) {
        // 後方外挿：最後の2点（y=9, y=10）を使用
        if (NUM_SAMPLES >= 2) {
            double y1 = MAX_Y - SAMPLE_INTERVAL;
            double y2 = MAX_Y;
            double x1 = state_.state[NUM_SAMPLES - 2];
            double x2 = state_.state[NUM_SAMPLES - 1];

            // 線形外挿
            return x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        } else {
            return state_.state[NUM_SAMPLES - 1];
        }
    }

    // 範囲内の線形補間
    double index = (y - MIN_Y) / SAMPLE_INTERVAL;
    int idx1 = static_cast<int>(std::floor(index));
    int idx2 = std::min(idx1 + 1, NUM_SAMPLES - 1);

    if (idx1 == idx2) {
        return state_.state[idx1];
    }

    double weight = index - idx1;
    return (1.0 - weight) * state_.state[idx1] + weight * state_.state[idx2];
}

std::vector<Eigen::Vector3d> LaneKalmanFilter::getUniformPoints(double interval) const {
    std::vector<Eigen::Vector3d> points;

    if (!state_.is_initialized) {
        return points;
    }

    for (double y = 0.0; y <= MAX_Y; y += interval) {
        double x = interpolatePosition(y);
        points.emplace_back(y, x, 0.0);  // (forward, lateral, z=0)
    }

    return points;
}

std::vector<double> LaneKalmanFilter::getLateralPositions() const {
    std::vector<double> positions;

    if (!state_.is_initialized) {
        return positions;
    }

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        positions.push_back(state_.state[i]);
    }

    return positions;
}

double LaneKalmanFilter::calculateMahalanobisDistance(const LaneObservation& observation) const {
    if (!state_.is_initialized) {
        return std::numeric_limits<double>::max();
    }

    auto [z, R] = createObservationVector(observation);
    if (z.size() == 0) {
        return std::numeric_limits<double>::max();
    }

    // 予測値を計算
    Eigen::VectorXd predicted(z.size());
    int obs_idx = 0;
    for (int i = 0; i < NUM_SAMPLES && obs_idx < z.size(); ++i) {
        double y = MIN_Y + i * SAMPLE_INTERVAL;

        // この y での観測があるかチェック
        for (size_t j = 0; j < observation.y_positions.size(); ++j) {
            if (std::abs(observation.y_positions[j] - y) < SAMPLE_INTERVAL * 0.5) {
                predicted[obs_idx] = state_.state[i];
                obs_idx++;
                break;
            }
        }
    }

    Eigen::VectorXd residual = z - predicted;
    return std::sqrt(residual.transpose() * R.inverse() * residual);
}

bool LaneKalmanFilter::isValidAssociation(const LaneObservation& observation, double max_mahalanobis) const {
    return calculateMahalanobisDistance(observation) < max_mahalanobis;
}

bool LaneKalmanFilter::checkGeometricConstraints(const LaneKalmanFilter& other_lane,
                                                double min_width, double max_width) const {
    if (!state_.is_initialized || !other_lane.state_.is_initialized) {
        return false;
    }

    // 複数点での車線幅をチェック
    for (int i = 0; i < NUM_SAMPLES; i += 2) {  // 2m間隔でチェック
        double width = std::abs(state_.state[i] - other_lane.state_.state[i]);
        if (width < min_width || width > max_width) {
            return false;
        }
    }

    return true;
}

void LaneKalmanFilter::updateConfidence(const LaneObservation& observation) {
    // 観測の信頼度と現在の信頼度を組み合わせて更新
    double alpha = 0.3;  // 更新重み
    state_.confidence = (1.0 - alpha) * state_.confidence + alpha * observation.confidence;
    state_.confidence = std::max(0.0, std::min(1.0, state_.confidence));
}

double LaneKalmanFilter::calculateObservationReliability(const LaneObservation& observation) const {
    if (!state_.is_initialized) {
        return 1.0;  // 初期化前は全て信頼
    }

    double total_change = 0.0;
    int valid_comparisons = 0;

    // 予測値と観測値の差分を計算
    for (size_t i = 0; i < observation.y_positions.size(); ++i) {
        double y = observation.y_positions[i];
        double observed_x = observation.x_positions[i];
        double predicted_x = interpolatePosition(y);

        double change = std::abs(observed_x - predicted_x);
        total_change += change;
        valid_comparisons++;
    }

    if (valid_comparisons == 0) {
        return 1.0;
    }

    double avg_change = total_change / valid_comparisons;

    // 急激変化の場合は信頼性を指数関数的に下げる
    if (avg_change > MAX_EXPECTED_CHANGE) {
        double reliability = std::exp(-(avg_change - MAX_EXPECTED_CHANGE) * RELIABILITY_DECAY_FACTOR);
        return std::max(0.01, reliability);  // 最小信頼性は1%
    }

    return 1.0;  // 正常な変化範囲内
}

void LaneKalmanFilter::reset() {
    state_.is_initialized = false;
    state_.confidence = 0.0;
    state_.state.setZero();
    state_.covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
}

// =============================================================================
// LaneKalmanManager Implementation
// =============================================================================

LaneKalmanManager::LaneKalmanManager() : next_track_id_(0) {
    last_update_ = std::chrono::steady_clock::now();
}

void LaneKalmanManager::updateLanes(const std::vector<LaneObservation>& observations) {
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_update_).count();
    dt = std::max(0.01, std::min(0.5, dt));  // 10ms～500ms でクランプ

    // 全トラックの予測
    predictAll(dt);

    // データアソシエーション
    auto associations = associateObservations(observations);

    // 既存トラックの更新
    for (const auto& [track_idx, obs_idx] : associations) {
        if (static_cast<size_t>(track_idx) < tracks_.size() && static_cast<size_t>(obs_idx) < observations.size()) {
            tracks_[track_idx]->update(observations[obs_idx]);
        }
    }

    // 未関連付けの観測から新しいトラックを作成
    std::vector<bool> obs_used(observations.size(), false);
    for (const auto& [track_idx, obs_idx] : associations) {
        if (static_cast<size_t>(obs_idx) < obs_used.size()) {
            obs_used[obs_idx] = true;
        }
    }

    for (size_t i = 0; i < observations.size(); ++i) {
        if (!obs_used[i] && observations[i].confidence > 0.3) {
            auto new_track = std::make_unique<LaneKalmanFilter>(next_track_id_++);
            new_track->initialize(observations[i]);
            tracks_.push_back(std::move(new_track));
        }
    }

    // 幾何的制約による検証
    validateLanePairs();

    // 古いトラックの削除
    pruneOldTracks();

    last_update_ = current_time;
}

void LaneKalmanManager::predictAll(double dt) {
    for (auto& track : tracks_) {
        track->predict(dt);
    }
}

std::vector<std::pair<int, int>>
LaneKalmanManager::associateObservations(const std::vector<LaneObservation>& observations) {
    std::vector<std::pair<int, int>> associations;

    if (tracks_.empty() || observations.empty()) {
        return associations;
    }

    // コスト行列を計算
    std::vector<std::vector<double>> cost_matrix(tracks_.size(),
                                               std::vector<double>(observations.size()));

    for (size_t i = 0; i < tracks_.size(); ++i) {
        for (size_t j = 0; j < observations.size(); ++j) {
            if (tracks_[i]->isValidAssociation(observations[j])) {
                cost_matrix[i][j] = tracks_[i]->calculateMahalanobisDistance(observations[j]);
            } else {
                cost_matrix[i][j] = std::numeric_limits<double>::max();
            }
        }
    }

    // 簡単な最近傍マッチング（Hungarian algorithm の代替）
    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> obs_used(observations.size(), false);

    for (size_t iteration = 0; iteration < std::min(tracks_.size(), observations.size()); ++iteration) {
        double min_cost = std::numeric_limits<double>::max();
        int best_track = -1;
        int best_obs = -1;

        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (track_used[i]) continue;
            for (size_t j = 0; j < observations.size(); ++j) {
                if (obs_used[j]) continue;
                if (cost_matrix[i][j] < min_cost) {
                    min_cost = cost_matrix[i][j];
                    best_track = i;
                    best_obs = j;
                }
            }
        }

        if (best_track != -1 && best_obs != -1 && min_cost < 9.0) {  // 3σ threshold
            associations.emplace_back(best_track, best_obs);
            track_used[best_track] = true;
            obs_used[best_obs] = true;
        } else {
            break;
        }
    }

    return associations;
}

void LaneKalmanManager::validateLanePairs() {
    // 信頼度の低いトラックを除去し、幾何的制約をチェック
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if ((*it)->getConfidence() < 0.1) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }

    // 左右レーンの整合性チェック（簡易版）
    if (tracks_.size() >= 2) {
        std::sort(tracks_.begin(), tracks_.end(),
                 [](const std::unique_ptr<LaneKalmanFilter>& a,
                    const std::unique_ptr<LaneKalmanFilter>& b) {
                     auto pos_a = a->getLateralPositions();
                     auto pos_b = b->getLateralPositions();
                     if (!pos_a.empty() && !pos_b.empty()) {
                         return pos_a[0] < pos_b[0];  // 横位置でソート
                     }
                     return false;
                 });

        // 隣接するレーン間で車線幅をチェック
        for (size_t i = 0; i < tracks_.size() - 1; ++i) {
            if (!tracks_[i]->checkGeometricConstraints(*tracks_[i + 1], 0.3, 0.5)) {
                // 制約違反の場合、信頼度の低い方を削除
                if (tracks_[i]->getConfidence() < tracks_[i + 1]->getConfidence()) {
                    tracks_.erase(tracks_.begin() + i);
                } else {
                    tracks_.erase(tracks_.begin() + i + 1);
                }
                break;
            }
        }
    }
}

void LaneKalmanManager::pruneOldTracks(double max_age_seconds) {
    auto current_time = std::chrono::steady_clock::now();

    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                      [current_time, max_age_seconds](const std::unique_ptr<LaneKalmanFilter>& track) {
                          // Note: This is a simplified check. In practice, you'd need to store last_update in the track
                          return track->getConfidence() < 0.05;
                      }),
        tracks_.end());
}

LaneKalmanManager::LaneType LaneKalmanManager::classifyLane(const LaneKalmanFilter& track) const {
    auto positions = track.getLateralPositions();
    if (positions.empty()) {
        return LaneType::UNKNOWN;
    }

    double avg_lateral = 0.0;
    for (double pos : positions) {
        avg_lateral += pos;
    }
    avg_lateral /= positions.size();

    if (avg_lateral < -0.2) {
        return LaneType::LEFT;
    } else if (avg_lateral > 0.2) {
        return LaneType::RIGHT;
    } else {
        return LaneType::CENTER;
    }
}

bool LaneKalmanManager::hasLeftLane() const {
    for (const auto& track : tracks_) {
        if (classifyLane(*track) == LaneType::LEFT && track->getConfidence() > 0.3) {
            return true;
        }
    }
    return false;
}

bool LaneKalmanManager::hasRightLane() const {
    for (const auto& track : tracks_) {
        if (classifyLane(*track) == LaneType::RIGHT && track->getConfidence() > 0.3) {
            return true;
        }
    }
    return false;
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getLeftLanePoints(double interval) const {
    for (const auto& track : tracks_) {
        if (classifyLane(*track) == LaneType::LEFT && track->getConfidence() > 0.3) {
            return track->getUniformPoints(interval);
        }
    }
    return {};
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getRightLanePoints(double interval) const {
    for (const auto& track : tracks_) {
        if (classifyLane(*track) == LaneType::RIGHT && track->getConfidence() > 0.3) {
            return track->getUniformPoints(interval);
        }
    }
    return {};
}

std::vector<Eigen::Vector3d> LaneKalmanManager::getCenterLanePoints(double interval) const {
    auto left = getLeftLanePoints(interval);
    auto right = getRightLanePoints(interval);

    if (left.empty() || right.empty()) {
        return {};
    }

    std::vector<Eigen::Vector3d> center;
    size_t min_size = std::min(left.size(), right.size());

    for (size_t i = 0; i < min_size; ++i) {
        Eigen::Vector3d center_point;
        center_point.x() = (left[i].x() + right[i].x()) * 0.5;
        center_point.y() = (left[i].y() + right[i].y()) * 0.5;
        center_point.z() = 0.0;
        center.push_back(center_point);
    }

    return center;
}

std::vector<std::vector<Eigen::Vector3d>> LaneKalmanManager::getFilteredLanes(double interval) const {
    std::vector<std::vector<Eigen::Vector3d>> lanes;

    auto left = getLeftLanePoints(interval);
    auto right = getRightLanePoints(interval);
    auto center = getCenterLanePoints(interval);

    if (!left.empty()) lanes.push_back(left);
    if (!right.empty()) lanes.push_back(right);
    if (!center.empty()) lanes.push_back(center);

    return lanes;
}

}  // namespace yolopnav