#include "pose_estimater/pose_estimater_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utilities/utils.hpp"
#include "vectormap_msgs/msg/line_string.hpp"

namespace pose_estimater
{
namespace
{

using LineString = vectormap_msgs::msg::LineString;

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.6943799901414e-3;
constexpr double HALF_PI = M_PI * 0.5;
constexpr double MIN_ICP_EIGENVALUE_RATIO = 1e-4;

IcpConfig make_icp_config(rclcpp::Node& node)
{
    IcpConfig config;
    config.max_iterations = node.get_parameter("icp.max_iterations").as_int();
    config.max_correspondence_distance = node.get_parameter("icp.max_correspondence_distance").as_double();
    config.convergence_translation_epsilon = node.get_parameter("icp.convergence_translation_epsilon").as_double();
    config.min_correspondences = static_cast<std::size_t>(node.get_parameter("icp.min_correspondences").as_int());
    config.max_mean_error = node.get_parameter("icp.max_mean_error").as_double();
    return config;
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

}

PoseEstimaterNode::PoseEstimaterNode(const rclcpp::NodeOptions& options)
: PoseEstimaterNode("", options)
{
}

PoseEstimaterNode::PoseEstimaterNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("pose_estimater_node", name_space, options),
  interval_ms_(get_parameter("interval_ms").as_int()),
  input_timeout_s_(get_parameter("input_timeout_s").as_double()),
  map_origin_lat_(get_parameter("map_origin_geodetic.latitude").as_double()),
  map_origin_lon_(get_parameter("map_origin_geodetic.longitude").as_double()),
  map_yaw_from_east_(get_parameter("map_yaw_from_east").as_double()),
  min_observed_points_(static_cast<std::size_t>(get_parameter("min_observed_points").as_int())),
  map_sample_interval_m_(get_parameter("map_sample_interval_m").as_double()),
  gnss_position_variance_(get_parameter("gnss_position_variance").as_double()),
  imu_yaw_variance_(get_parameter("imu_yaw_variance").as_double()),
  icp_position_variance_(get_parameter("icp_position_variance").as_double()),
  icp_matcher_(make_icp_config(*this)),
  has_last_lane_line_update_stamp_(false)
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

    icp_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/icp_pose", rclcpp::SensorDataQoS().keep_last(1));
    raw_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose_raw", rclcpp::SensorDataQoS().keep_last(1));
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

bool PoseEstimaterNode::rebuild_map_points(const vectormap_msgs::msg::VectorMap& map_msg)
{
    std::vector<IcpMapPoint> points;
    for (const auto& line_string : map_msg.line_strings) {
        if (!line_string.is_observable || line_string.marking_type == LineString::MARKING_VIRTUAL) {
            continue;
        }
        for (std::size_t i = 1U; i < line_string.points.size(); ++i) {
            const Eigen::Vector2d start(line_string.points[i - 1U].x, line_string.points[i - 1U].y);
            const Eigen::Vector2d end(line_string.points[i].x, line_string.points[i].y);
            const Eigen::Vector2d delta = end - start;
            const double length = delta.norm();
            if (length <= 0.0) {
                continue;
            }
            const Eigen::Vector2d direction = delta / length;
            const Eigen::Vector2d normal(-direction.y(), direction.x());
            const int samples = std::max(1, static_cast<int>(std::ceil(length / map_sample_interval_m_)));
            for (int sample = 0; sample <= samples; ++sample) {
                const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
                points.push_back(IcpMapPoint{start + ratio * delta, normal});
            }
        }
    }

    auto target_map = std::make_shared<IcpTargetMap>(std::move(points));
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_points_ = target_map;
    return true;
}

bool PoseEstimaterNode::gnss_to_map_pose(
    const sensor_msgs::msg::NavSatFix& gnss_msg,
    const sensor_msgs::msg::Imu& imu_msg,
    geometry_msgs::msg::PoseWithCovarianceStamped& pose_out)
{
    if (gnss_msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        return false;
    }
    if (!std::isfinite(gnss_msg.latitude) || !std::isfinite(gnss_msg.longitude)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "GNSSの緯度または経度が非有限値のためこの観測をスキップする");
        return false;
    }

    const double lat0_rad = utils::dtor(map_origin_lat_);
    const double sin_lat0 = std::sin(lat0_rad);
    const double sin2_lat0 = sin_lat0 * sin_lat0;
    const double denom = std::sqrt(1.0 - WGS84_E2 * sin2_lat0);
    const double N = WGS84_A / denom;
    const double M = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);

    const double delta_lat = utils::dtor(gnss_msg.latitude - map_origin_lat_);
    const double delta_lon = utils::dtor(gnss_msg.longitude - map_origin_lon_);
    const double north = M * delta_lat;
    const double east = N * std::cos(lat0_rad) * delta_lon;

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

std::vector<Eigen::Vector2d> PoseEstimaterNode::lane_line_points_from_cloud(
    const sensor_msgs::msg::PointCloud2& cloud)
{
    std::vector<Eigen::Vector2d> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        points.emplace_back(static_cast<double>(*iter_x), static_cast<double>(*iter_y));
    }
    return points;
}

std::vector<Eigen::Vector2d> PoseEstimaterNode::observed_points_in_map(
    const std::vector<Eigen::Vector2d>& base_points,
    const double x, const double y, const double yaw) const
{
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const Eigen::Vector2d translation(x, y);

    std::vector<Eigen::Vector2d> map_points;
    map_points.reserve(base_points.size());
    for (const auto& base_point : base_points) {
        const Eigen::Vector2d rotated(
            cos_yaw * base_point.x() - sin_yaw * base_point.y(),
            sin_yaw * base_point.x() + cos_yaw * base_point.y());
        map_points.push_back(translation + rotated);
    }
    return map_points;
}

Eigen::Matrix2d PoseEstimaterNode::icp_measurement_covariance(const IcpResult& result) const
{
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
        result.normal_matrix / static_cast<double>(result.correspondences));
    const double max_eigenvalue = solver.eigenvalues()(1);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2; ++i) {
        const double ratio =
            std::max(solver.eigenvalues()(i) / max_eigenvalue, MIN_ICP_EIGENVALUE_RATIO);
        const Eigen::Vector2d direction = solver.eigenvectors().col(i);
        covariance += (icp_position_variance_ / ratio) *
            direction * direction.transpose();
    }
    return covariance;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseEstimaterNode::make_icp_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose,
    const double x, const double y, const Eigen::Matrix2d& position_covariance) const
{
    geometry_msgs::msg::PoseWithCovarianceStamped icp_pose = raw_pose;
    icp_pose.pose.pose.position.x = x;
    icp_pose.pose.pose.position.y = y;
    fill_pose_covariance(icp_pose.pose.covariance, position_covariance, imu_yaw_variance_);
    return icp_pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseEstimaterNode::fallback_icp_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose) const
{
    return make_icp_pose(
        raw_pose,
        raw_pose.pose.pose.position.x,
        raw_pose.pose.pose.position.y,
        Eigen::Matrix2d::Identity() * gnss_position_variance_);
}

void PoseEstimaterNode::timer_callback()
{
    sensor_msgs::msg::PointCloud2::SharedPtr lane_line_points_msg;
    sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    std::shared_ptr<const IcpTargetMap> map_points;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        lane_line_points_msg = latest_lane_line_points_;
        gnss_msg = latest_gnss_msg_;
        imu_msg = latest_imu_msg_;
        map_points = map_points_;
    }

    if (!gnss_msg || !imu_msg) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "waiting for GNSS and IMU");
        return;
    }

    const rclcpp::Time current_time = this->now();
    const rclcpp::Time gnss_stamp(gnss_msg->header.stamp, current_time.get_clock_type());
    const rclcpp::Time imu_stamp(imu_msg->header.stamp, current_time.get_clock_type());
    if ((current_time - gnss_stamp).seconds() > input_timeout_s_ ||
        (current_time - imu_stamp).seconds() > input_timeout_s_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "GNSS/IMU が %.1fs 以上途絶しているため自己位置のpublishを停止する",
            input_timeout_s_);
        return;
    }

    try {
        geometry_msgs::msg::PoseWithCovarianceStamped raw_pose;
        if (!gnss_to_map_pose(*gnss_msg, *imu_msg, raw_pose)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "GNSS fix not available");
            return;
        }
        raw_pose_publisher_->publish(raw_pose);

        bool has_new_lane_line = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            has_new_lane_line = lane_line_points_msg &&
                (!has_last_lane_line_update_stamp_ ||
                 lane_line_points_msg->header.stamp.sec != last_lane_line_update_stamp_.sec ||
                 lane_line_points_msg->header.stamp.nanosec != last_lane_line_update_stamp_.nanosec);
            if (has_new_lane_line) {
                has_last_lane_line_update_stamp_ = true;
                last_lane_line_update_stamp_ = lane_line_points_msg->header.stamp;
            }
        }

        if (!has_new_lane_line || !map_points || map_points->empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "waiting for a new lane line scan and vector map");
            icp_pose_publisher_->publish(fallback_icp_pose(raw_pose));
            return;
        }

        const auto base_points = lane_line_points_from_cloud(*lane_line_points_msg);
        const double raw_yaw = utils::yaw_from_quaternion(raw_pose.pose.pose.orientation);
        const auto source_points = observed_points_in_map(
            base_points, raw_pose.pose.pose.position.x, raw_pose.pose.pose.position.y, raw_yaw);

        if (base_points.size() < min_observed_points_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "not enough observed lane points: %zu", base_points.size());
            icp_pose_publisher_->publish(fallback_icp_pose(raw_pose));
            return;
        }

        const auto result = icp_matcher_.align_translation_only(source_points, *map_points);
        if (!result.converged) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "ICP failed: correspondences=%zu, mean_error=%.3f",
                result.correspondences, result.mean_error);
            icp_pose_publisher_->publish(fallback_icp_pose(raw_pose));
            return;
        }

        const double icp_x = raw_pose.pose.pose.position.x + result.translation.x();
        const double icp_y = raw_pose.pose.pose.position.y + result.translation.y();
        icp_pose_publisher_->publish(
            make_icp_pose(raw_pose, icp_x, icp_y, icp_measurement_covariance(result)));

        RCLCPP_DEBUG(
            this->get_logger(),
            "localized correction=(%.3f, %.3f), correspondences=%zu, mean_error=%.3f",
            result.translation.x(), result.translation.y(),
            result.correspondences, result.mean_error);
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "pose estimation skipped: %s", error.what());
    }
}

}
