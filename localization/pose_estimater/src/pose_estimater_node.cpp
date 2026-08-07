#include "pose_estimater/pose_estimater_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <random>
#include <utility>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utilities/utils.hpp"
#include "utilities/vectornav_frame.hpp"
#include "vectormap_msgs/msg/line_string.hpp"

namespace pose_estimater
{
namespace
{

using LineString = vectormap_msgs::msg::LineString;

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.6943799901414e-3;
constexpr double HALF_PI = M_PI * 0.5;

ParticleFilterConfig make_particle_filter_config(rclcpp::Node& node)
{
    ParticleFilterConfig config;
    config.num_particles = static_cast<std::size_t>(node.get_parameter("num_particles").as_int());
    config.odom_fw_dev_per_fw =
        node.get_parameter("odom_fw_dev_per_fw").as_double();
    config.odom_fw_dev_per_rot =
        node.get_parameter("odom_fw_dev_per_rot").as_double();
    config.odom_rot_dev_per_fw =
        node.get_parameter("odom_rot_dev_per_fw").as_double();
    config.odom_rot_dev_per_rot =
        node.get_parameter("odom_rot_dev_per_rot").as_double();
    config.likelihood_dev = node.get_parameter("likelihood_dev").as_double();
    config.likelihood_max_dist =
        node.get_parameter("likelihood_max_dist").as_double();
    config.resample_ess_ratio_threshold =
        node.get_parameter("resample_ess_ratio_threshold").as_double();
    config.reinit_residual_threshold =
        node.get_parameter("reinit_residual_threshold").as_double();
    config.reinit_consecutive_frames =
        static_cast<int>(node.get_parameter("reinit_consecutive_frames").as_int());
    config.min_position_variance = node.get_parameter("min_position_variance").as_double();
    config.min_yaw_variance = node.get_parameter("min_yaw_variance").as_double();
    return config;
}

double meters_per_rad_latitude(const double lat0_rad)
{
    const double sin_lat0 = std::sin(lat0_rad);
    const double denom = std::sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0);
    return WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);
}

double meters_per_rad_longitude(const double lat0_rad)
{
    const double sin_lat0 = std::sin(lat0_rad);
    return WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat0 * sin_lat0) * std::cos(lat0_rad);
}

double normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void fill_pose_covariance(
    std::array<double, 36>& covariance,
    const Eigen::Matrix2d& position_covariance,
    const double yaw_variance)
{
    covariance.fill(0.0);
    covariance[0] = position_covariance(0, 0);
    covariance[1] = position_covariance(0, 1);
    covariance[6] = position_covariance(1, 0);
    covariance[7] = position_covariance(1, 1);
    covariance[35] = yaw_variance;
}

std::vector<Eigen::Vector2d> lane_line_points_from_cloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    std::vector<Eigen::Vector2d> points;
    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            points.emplace_back(static_cast<double>(*iter_x), static_cast<double>(*iter_y));
        }
    } catch (const std::exception&) {
        return {};
    }
    return points;
}

}  // namespace

PoseEstimaterNode::PoseEstimaterNode(const rclcpp::NodeOptions& options)
: PoseEstimaterNode("", options)
{
}

PoseEstimaterNode::PoseEstimaterNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("pose_estimater_node", name_space, options),
  interval_ms_(get_parameter("interval_ms").as_int()),
  map_origin_lat_(get_parameter("map_origin_geodetic.latitude").as_double()),
  map_origin_lon_(get_parameter("map_origin_geodetic.longitude").as_double()),
  map_yaw_from_east_(get_parameter("map_yaw_from_east").as_double()),
  meters_per_rad_lat_(meters_per_rad_latitude(utils::dtor(map_origin_lat_))),
  meters_per_rad_lon_(meters_per_rad_longitude(utils::dtor(map_origin_lat_))),
  min_observed_points_(static_cast<std::size_t>(get_parameter("min_observed_points").as_int())),
  map_sample_interval_m_(get_parameter("map_sample_interval_m").as_double()),
  gnss_position_variance_(get_parameter("gnss_position_variance").as_double()),
  imu_yaw_variance_(get_parameter("imu_yaw_variance").as_double()),
  particle_filter_(make_particle_filter_config(*this), std::random_device{}())
{
    lane_line_points_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/lane_line_points", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PoseEstimaterNode::lane_line_points_callback, this, std::placeholders::_1));
    vector_map_subscription_ = this->create_subscription<vectormap_msgs::msg::VectorMap>(
        "/vector_map", rclcpp::QoS(1).transient_local(),
        std::bind(&PoseEstimaterNode::vector_map_callback, this, std::placeholders::_1));
    gnss_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/vectornav/gnss", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PoseEstimaterNode::gnss_callback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/vectornav/imu", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PoseEstimaterNode::imu_callback, this, std::placeholders::_1));
    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PoseEstimaterNode::velocity_callback, this, std::placeholders::_1));
    initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1),
        std::bind(&PoseEstimaterNode::initial_pose_callback, this, std::placeholders::_1));

    pf_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pf_pose", rclcpp::SensorDataQoS().keep_last(1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms_),
        std::bind(&PoseEstimaterNode::timer_callback, this));
}

void PoseEstimaterNode::lane_line_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_lane_line_points_ = msg;
}

void PoseEstimaterNode::vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    rebuild_map_points(*msg);
}

void PoseEstimaterNode::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_gnss_msg_ = msg;
}

void PoseEstimaterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_msg_ = msg;
}

void PoseEstimaterNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_msg_ = msg;
}

void PoseEstimaterNode::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_initial_pose_ = msg;
}

void PoseEstimaterNode::rebuild_map_points(const vectormap_msgs::msg::VectorMap& map_msg)
{
    std::vector<PfMapPoint> points;
    for (const auto& line_string : map_msg.line_strings) {
        if (!line_string.is_observable || line_string.marking_type == LineString::MARKING_VIRTUAL) {
            continue;
        }
        for (std::size_t i = 1U; i < line_string.points.size(); ++i) {
            const Eigen::Vector2d start(line_string.points[i - 1U].x, line_string.points[i - 1U].y);
            const Eigen::Vector2d end(line_string.points[i].x, line_string.points[i].y);
            const double length = (end - start).norm();
            const int samples = std::max(1, static_cast<int>(std::ceil(length / map_sample_interval_m_)));
            for (int sample = 0; sample <= samples; ++sample) {
                const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
                points.push_back(PfMapPoint{start + ratio * (end - start)});
            }
        }
    }

    auto target_map = std::make_shared<PfTargetMap>(std::move(points));
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_points_ = target_map;
}

bool PoseEstimaterNode::gnss_to_map_pose(
    const sensor_msgs::msg::NavSatFix& gnss_msg,
    const sensor_msgs::msg::Imu& imu_msg,
    geometry_msgs::msg::PoseWithCovarianceStamped& pose_out) const
{
    if (gnss_msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        return false;
    }

    const double north = meters_per_rad_lat_ * utils::dtor(gnss_msg.latitude - map_origin_lat_);
    const double east = meters_per_rad_lon_ * utils::dtor(gnss_msg.longitude - map_origin_lon_);

    const double cos_yaw = std::cos(map_yaw_from_east_);
    const double sin_yaw = std::sin(map_yaw_from_east_);
    const double x_map = cos_yaw * east + sin_yaw * north;
    const double y_map = -sin_yaw * east + cos_yaw * north;

    const double yaw_enu = normalize_angle(HALF_PI + utils::yaw_from_quaternion(imu_msg.orientation));
    const double yaw_map = normalize_angle(yaw_enu - map_yaw_from_east_);

    pose_out.header.stamp = gnss_msg.header.stamp;
    pose_out.header.frame_id = "map";
    pose_out.pose.pose.position.x = x_map;
    pose_out.pose.pose.position.y = y_map;
    pose_out.pose.pose.position.z = 0.0;
    pose_out.pose.pose.orientation = utils::yaw_to_quaternion(yaw_map);
    fill_pose_covariance(
        pose_out.pose.covariance,
        Eigen::Matrix2d::Identity() * gnss_position_variance_,
        imu_yaw_variance_);

    return true;
}

void PoseEstimaterNode::initialize_particle_filter(
    const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose)
{
    const double raw_yaw = utils::yaw_from_quaternion(raw_pose.pose.pose.orientation);
    particle_filter_.initialize(
        raw_pose.pose.pose.position.x, raw_pose.pose.pose.position.y, raw_yaw,
        std::sqrt(gnss_position_variance_), std::sqrt(imu_yaw_variance_));
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseEstimaterNode::make_pose(
    const builtin_interfaces::msg::Time& stamp, const PoseEstimate2D& estimate) const
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = estimate.x;
    pose.pose.pose.position.y = estimate.y;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = utils::yaw_to_quaternion(estimate.yaw);
    fill_pose_covariance(pose.pose.covariance, estimate.position_covariance, estimate.yaw_variance);
    return pose;
}

void PoseEstimaterNode::timer_callback()
{
    sensor_msgs::msg::PointCloud2::SharedPtr lane_line_points_msg;
    sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr velocity_msg;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg;
    std::shared_ptr<const PfTargetMap> map_points;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        lane_line_points_msg = latest_lane_line_points_;
        gnss_msg = latest_gnss_msg_;
        imu_msg = latest_imu_msg_;
        velocity_msg = latest_velocity_msg_;
        map_points = map_points_;
        initial_pose_msg = pending_initial_pose_;
        pending_initial_pose_.reset();
    }

    if (initial_pose_msg) {
        initial_pose_msg->header.frame_id = "map";
        initial_pose_msg->header.stamp = this->get_clock()->now();
        initialize_particle_filter(*initial_pose_msg);
        pf_pose_publisher_->publish(*initial_pose_msg);
        return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped raw_pose;
    const bool has_raw_pose =
        gnss_msg && imu_msg && gnss_to_map_pose(*gnss_msg, *imu_msg, raw_pose);

    if (!particle_filter_.initialized()) {
        if (!has_raw_pose) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "waiting for GNSS and IMU");
            return;
        }
        if (!map_points || map_points->empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "waiting for vector map");
            pf_pose_publisher_->publish(raw_pose);
            return;
        }
        initialize_particle_filter(raw_pose);
        pf_pose_publisher_->publish(raw_pose);
        return;
    }

    if (velocity_msg) {
        // velocity_bodyはVN body系(x前 / y右 / z下)で来るのでREP-103へ直す。
        const geometry_msgs::msg::Twist twist =
            utils::vn_body_to_rep103(velocity_msg->twist.twist);
        const double dt = static_cast<double>(interval_ms_) / 1000.0;
        particle_filter_.predict(twist.linear.x, twist.angular.z, dt);
    }

    const bool has_new_lane_line =
        lane_line_points_msg && lane_line_points_msg != processed_lane_line_points_;
    processed_lane_line_points_ = lane_line_points_msg;

    builtin_interfaces::msg::Time estimate_stamp = this->get_clock()->now();

    if (has_new_lane_line && map_points && !map_points->empty()) {
        auto source_points = lane_line_points_from_cloud(*lane_line_points_msg);
        if (source_points.size() >= min_observed_points_) {
            particle_filter_.update_weights(source_points, *map_points);
            estimate_stamp = lane_line_points_msg->header.stamp;

            if (particle_filter_.needs_reinitialization() && has_raw_pose) {
                initialize_particle_filter(raw_pose);
            } else if (particle_filter_.should_resample()) {
                particle_filter_.resample();
            }
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "not enough observed lane points: %zu", source_points.size());
        }
    }

    pf_pose_publisher_->publish(
        make_pose(estimate_stamp, particle_filter_.estimate()));
}

}  // namespace pose_estimater
