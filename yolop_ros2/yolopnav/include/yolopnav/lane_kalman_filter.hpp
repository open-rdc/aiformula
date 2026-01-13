#pragma once

#include <Eigen/Dense>
#include <vector>

namespace yolopnav {

class LaneKalmanFilter {
public:
    LaneKalmanFilter();

    void update(const std::vector<Eigen::Vector3d>& points);

    void predict();

    double evaluate(double x) const;

    bool isInitialized() const { return initialized_; }

    Eigen::Vector4d getState() const { return state_; }

private:
    Eigen::Vector4d state_;           // [a0, a1, a2, a3]^T
    Eigen::Matrix4d covariance_;      // P
    Eigen::Matrix4d process_noise_;   // Q
    double observation_noise_;        // R
    bool initialized_;

    void initialize(const std::vector<Eigen::Vector3d>& points);
};

class LaneKalmanManager {
public:
    LaneKalmanManager();

    void updateLeft(const std::vector<Eigen::Vector3d>& points);
    void updateRight(const std::vector<Eigen::Vector3d>& points);
    void updateCenter(const std::vector<Eigen::Vector3d>& points);

    void predict();

    bool hasLeftLane() const { return left_filter_.isInitialized(); }
    bool hasRightLane() const { return right_filter_.isInitialized(); }
    bool hasCenterLane() const { return center_filter_.isInitialized(); }

    std::vector<Eigen::Vector3d> getLeftLanePoints(double interval, double min_x = 0.0, double max_x = 10.0) const;
    std::vector<Eigen::Vector3d> getRightLanePoints(double interval, double min_x = 0.0, double max_x = 10.0) const;
    std::vector<Eigen::Vector3d> getCenterLanePoints(double interval, double min_x = 0.0, double max_x = 10.0) const;

private:
    LaneKalmanFilter left_filter_;
    LaneKalmanFilter right_filter_;
    LaneKalmanFilter center_filter_;

    std::vector<Eigen::Vector3d> generatePoints(const LaneKalmanFilter& filter,
                                                 double interval, double min_x, double max_x) const;
};

}  // namespace yolopnav
