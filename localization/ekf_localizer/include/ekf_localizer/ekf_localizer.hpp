#pragma once

#include <deque>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ekf_localizer
{

struct DelayGateResult
{
    double delay_time_s;
    bool passed;
};

double mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance);
DelayGateResult check_delay_gate(
    const rclcpp::Time& now,
    const rclcpp::Time& stamp,
    double max_delay_s);

struct EkfLocalizerConfig
{
    double initial_position_variance;
    double initial_yaw_variance;
    double process_position_variance;
    double process_yaw_variance;
    double process_velocity_variance;
    double process_yaw_rate_variance;
    double position_gate_dist;
    double yaw_gate_dist;
};

class EkfLocalizer
{
public:
    explicit EkfLocalizer(const EkfLocalizerConfig& config);

    bool initialized() const;
    void initialize(double x, double y, double yaw, const rclcpp::Time& stamp);
    void predict(double velocity, double yaw_rate, const rclcpp::Time& stamp);
    bool update_position(
        double x, double y, const Eigen::Matrix2d& covariance, const rclcpp::Time& stamp);
    bool update_yaw(double yaw, double variance, const rclcpp::Time& stamp);
    bool pose_at(const rclcpp::Time& stamp, double& x, double& y, double& yaw) const;

    geometry_msgs::msg::PoseWithCovarianceStamped make_pose(const std::string& frame_id) const;

private:
    struct HistoryEntry
    {
        rclcpp::Time stamp;
        double x;
        double y;
        double yaw;
    };

    void record_history();
    bool apply_position_update(const Eigen::Vector2d& residual, const Eigen::Matrix2d& covariance);
    bool apply_yaw_update(double residual, double variance);

    EkfLocalizerConfig config_;
    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    rclcpp::Time stamp_;
    bool initialized_;
    std::deque<HistoryEntry> history_;
};

}
