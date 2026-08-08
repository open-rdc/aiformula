#pragma once

#include "pose_estimater/particle_filter.hpp"
#include "pose_estimater/visibility_control.h"

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vectormap_msgs/msg/vector_map.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace pose_estimater {

class PoseEstimaterNode : public rclcpp::Node {
   public:
    POSE_ESTIMATER_PUBLIC
    explicit PoseEstimaterNode (const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

    POSE_ESTIMATER_PUBLIC
    explicit PoseEstimaterNode (const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

   private:
    void lane_line_points_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void vector_map_callback (const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void gnss_callback (const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg);
    void velocity_callback (const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void initial_pose_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timer_callback ();

    void rebuild_map_points (const vectormap_msgs::msg::VectorMap &map_msg);

    bool gnss_to_map_pose (const sensor_msgs::msg::NavSatFix &gnss_msg, const sensor_msgs::msg::Imu &imu_msg, geometry_msgs::msg::PoseWithCovarianceStamped &pose_out) const;

    void initialize_particle_filter (const geometry_msgs::msg::PoseWithCovarianceStamped &raw_pose);

    geometry_msgs::msg::PoseWithCovarianceStamped make_pose (const builtin_interfaces::msg::Time &stamp, const PoseEstimate2D &estimate) const;

    void publish_particle_pose_array (const builtin_interfaces::msg::Time &stamp) const;

    const int         interval_ms_;
    const double      map_origin_lat_;
    const double      map_origin_lon_;
    const double      map_yaw_from_east_;
    const double      meters_per_rad_lat_;
    const double      meters_per_rad_lon_;
    const std::size_t min_observed_points_;
    const double      map_sample_interval_m_;
    const double      gnss_position_variance_;
    const double      imu_yaw_variance_;
    ParticleFilter    particle_filter_;

    std::shared_ptr<const PfTargetMap>                        map_points_;
    sensor_msgs::msg::PointCloud2::SharedPtr                  latest_lane_line_points_;
    sensor_msgs::msg::NavSatFix::SharedPtr                    latest_gnss_msg_;
    sensor_msgs::msg::Imu::SharedPtr                          latest_imu_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr latest_velocity_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr  pending_initial_pose_;
    std::mutex                                                data_mutex_;
    sensor_msgs::msg::PointCloud2::SharedPtr                  processed_lane_line_points_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                  lane_line_points_subscription_;
    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr                 vector_map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                    gnss_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                          imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  initial_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr     pf_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr                     particle_pose_array_publisher_;
    rclcpp::TimerBase::SharedPtr                                                    timer_;
};

}  // namespace pose_estimater
