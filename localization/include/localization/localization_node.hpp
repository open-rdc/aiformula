#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "localization/ekf_localizer.hpp"
#include "localization/icp_matching.hpp"
#include "localization/lane_pixel_to_point.hpp"
#include "localization/visibility_control.h"
#include "vectormap_msgs/msg/vector_map.hpp"

namespace localization
{

class LocalizationNode : public rclcpp::Node
{
public:
    LOCALIZATION_PUBLIC
    explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LOCALIZATION_PUBLIC
    explicit LocalizationNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg);
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void timer_callback();

    void rebuild_map_points(const vectormap_msgs::msg::VectorMap& map_msg);

    // lat/lon → ENU → map 座標変換。GNSS fix 未取得の場合は false を返す
    bool gnss_to_map_pose(
        const sensor_msgs::msg::NavSatFix& gnss_msg,
        const sensor_msgs::msg::Imu& imu_msg,
        geometry_msgs::msg::PoseWithCovarianceStamped& pose_out) const;
    geometry_msgs::msg::PoseWithCovarianceStamped update_ekf_with_raw_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose,
        const sensor_msgs::msg::Imu& imu_msg,
        const geometry_msgs::msg::TwistWithCovarianceStamped& velocity_msg);
    bool is_valid_velocity_frame(const std::string& frame_id) const;
    static bool same_stamp(
        const builtin_interfaces::msg::Time& lhs,
        const builtin_interfaces::msg::Time& rhs);

    std::vector<Eigen::Vector2d> observed_points_in_initial_map(
        const std::vector<Eigen::Vector2d>& base_points,
        const geometry_msgs::msg::PoseWithCovarianceStamped& initial_pose) const;
    geometry_msgs::msg::PoseWithCovarianceStamped update_ekf_with_icp_pose(double x, double y);
    visualization_msgs::msg::MarkerArray make_lane_line_marker_array(
        const std::vector<Eigen::Vector2d>& base_points) const;
    visualization_msgs::msg::MarkerArray make_lane_line_map_marker_array(
        const std::vector<Eigen::Vector2d>& map_points,
        const std::string& ns,
        float r, float g, float b) const;

    const int update_period_ms_;
    const std::string map_frame_id_;
    const std::string base_frame_id_;
    const std::string localized_pose_topic_;
    const std::string raw_pose_topic_;
    const std::string velocity_topic_;
    const std::string imu_yaw_convention_;
    const double map_origin_lat_;
    const double map_origin_lon_;
    const double map_yaw_from_east_;
    const uint8_t mask_threshold_;
    const int pixel_step_;
    const std::size_t max_observed_points_;
    const std::size_t min_observed_points_;
    const std::size_t min_map_points_;
    const double map_sample_interval_m_;
    const CameraModel camera_model_;
    const IcpMatcher icp_matcher_;
    const EkfLocalizerConfig ekf_config_;
    EkfLocalizer ekf_localizer_;
    const rclcpp::QoS qos_;

    std::shared_ptr<const IcpTargetMap> map_points_;
    sensor_msgs::msg::Image::SharedPtr latest_mask_image_;
    sensor_msgs::msg::NavSatFix::SharedPtr latest_gnss_msg_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr latest_velocity_msg_;
    mutable std::mutex data_mutex_;
    GroundProjectionLUT ground_projection_lut_;
    bool has_last_gnss_update_stamp_;
    bool has_last_imu_update_stamp_;
    builtin_interfaces::msg::Time last_gnss_update_stamp_;
    builtin_interfaces::msg::Time last_imu_update_stamp_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_subscription_;
    rclcpp::Subscription<vectormap_msgs::msg::VectorMap>::SharedPtr vector_map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localized_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr raw_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_line_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_line_map_raw_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_line_map_corrected_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace localization
