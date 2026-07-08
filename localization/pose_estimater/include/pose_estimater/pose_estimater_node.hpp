#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pose_estimater/icp_matching.hpp"
#include "pose_estimater/visibility_control.h"
#include "vectormap_msgs/msg/vector_map.hpp"

namespace pose_estimater
{

class PoseEstimaterNode : public rclcpp::Node
{
public:
    POSE_ESTIMATER_PUBLIC
    explicit PoseEstimaterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    POSE_ESTIMATER_PUBLIC
    explicit PoseEstimaterNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void lane_line_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timer_callback();

    bool rebuild_map_points(const vectormap_msgs::msg::VectorMap& map_msg);

    bool gnss_to_map_pose(
        const sensor_msgs::msg::NavSatFix& gnss_msg,
        const sensor_msgs::msg::Imu& imu_msg,
        geometry_msgs::msg::PoseWithCovarianceStamped& pose_out);

    static std::vector<Eigen::Vector2d> lane_line_points_from_cloud(
        const sensor_msgs::msg::PointCloud2& cloud);
    std::vector<Eigen::Vector2d> observed_points_in_map(
        const std::vector<Eigen::Vector2d>& base_points,
        double x, double y, double yaw) const;
    Eigen::Matrix2d icp_measurement_covariance(const IcpResult& result) const;
    geometry_msgs::msg::PoseWithCovarianceStamped make_icp_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose,
        double x, double y, const Eigen::Matrix2d& position_covariance) const;
    geometry_msgs::msg::PoseWithCovarianceStamped fallback_icp_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose) const;

    const int interval_ms_;
    const double input_timeout_s_;
    const double map_origin_lat_;
    const double map_origin_lon_;
    const double map_yaw_from_east_;
    const std::size_t min_observed_points_;
    const double map_sample_interval_m_;
    const double gnss_position_variance_;
    const double imu_yaw_variance_;
    const double icp_position_variance_;
    const IcpMatcher icp_matcher_;

    std::shared_ptr<const IcpTargetMap> map_points_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_lane_line_points_;
    sensor_msgs::msg::NavSatFix::SharedPtr latest_gnss_msg_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    mutable std::mutex data_mutex_;
    bool has_last_lane_line_update_stamp_;
    builtin_interfaces::msg::Time last_lane_line_update_stamp_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lane_line_points_subscription_;
    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr icp_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr raw_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
