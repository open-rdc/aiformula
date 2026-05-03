#pragma once

#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vectormap_localization
{

struct EkfLocalizerConfig
{
    double initial_position_variance;
    double initial_yaw_variance;
    double initial_velocity_variance;
    double initial_yaw_rate_variance;
    double process_position_variance;
    double process_yaw_variance;
    double process_velocity_variance;
    double process_yaw_rate_variance;
    double gnss_position_variance;
    double imu_yaw_variance;
    double icp_position_variance;
};

class EkfLocalizer
{
public:
    explicit EkfLocalizer(const EkfLocalizerConfig& config);

    bool initialized() const;
    void initialize(
        double x,
        double y,
        double yaw,
        double velocity,
        double yaw_rate,
        const rclcpp::Time& stamp);
    void predict(double velocity, double yaw_rate, const rclcpp::Time& stamp);
    void update_position(double x, double y, double variance);
    void update_yaw(double yaw, double variance);

    geometry_msgs::msg::PoseWithCovarianceStamped make_pose(
        const rclcpp::Time& stamp,
        const std::string& frame_id) const;

private:
    static double normalize_angle(double angle);
    static void set_yaw(geometry_msgs::msg::Quaternion& quaternion, double yaw);
    static void validate_variance(double variance, const char* name);

    EkfLocalizerConfig config_;
    Eigen::Matrix<double, 5, 1> state_;
    Eigen::Matrix<double, 5, 5> covariance_;
    rclcpp::Time stamp_;
    bool initialized_;
};

}  // namespace vectormap_localization
