#include "vectormap_matching/vectormap_localization_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vectormap_msgs/msg/line_string.hpp"

namespace vectormap_matching
{
namespace
{

using LineString = vectormap_msgs::msg::LineString;

// WGS84 楕円体定数
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.6943799901414e-3;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double HALF_PI = M_PI * 0.5;

CameraModel make_camera_model(rclcpp::Node& node)
{
    CameraModel camera_model;
    camera_model.fx = node.get_parameter("camera.fx").as_double();
    camera_model.fy = node.get_parameter("camera.fy").as_double();
    camera_model.cx = node.get_parameter("camera.cx").as_double();
    camera_model.cy = node.get_parameter("camera.cy").as_double();
    camera_model.ground_plane_z_base = node.get_parameter("ground_plane_z_base").as_double();
    camera_model.min_ground_intersection_distance =
        node.get_parameter("min_ground_intersection_distance").as_double();
    camera_model.max_ground_intersection_distance =
        node.get_parameter("max_ground_intersection_distance").as_double();

    if (!std::isfinite(camera_model.fx) ||
        !std::isfinite(camera_model.fy) ||
        !std::isfinite(camera_model.cx) ||
        !std::isfinite(camera_model.cy) ||
        !std::isfinite(camera_model.ground_plane_z_base) ||
        !std::isfinite(camera_model.min_ground_intersection_distance) ||
        !std::isfinite(camera_model.max_ground_intersection_distance))
    {
        throw std::invalid_argument("camera and ground plane parameters must be finite");
    }
    if (camera_model.fx <= 0.0 || camera_model.fy <= 0.0) {
        throw std::invalid_argument("camera.fx and camera.fy must be greater than 0");
    }
    if (camera_model.min_ground_intersection_distance < 0.0 ||
        camera_model.max_ground_intersection_distance <= camera_model.min_ground_intersection_distance)
    {
        throw std::invalid_argument("ground intersection distance parameters are invalid");
    }

    const double roll = node.get_parameter("camera_to_base.roll").as_double();
    const double pitch = node.get_parameter("camera_to_base.pitch").as_double();
    const double yaw = node.get_parameter("camera_to_base.yaw").as_double();
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
        throw std::invalid_argument("camera_to_base rotation parameters must be finite");
    }
    camera_model.camera_to_base_rotation = rotation_matrix_from_rpy(roll, pitch, yaw);
    camera_model.camera_to_base_translation = Eigen::Vector3d(
        node.get_parameter("camera_to_base.x").as_double(),
        node.get_parameter("camera_to_base.y").as_double(),
        node.get_parameter("camera_to_base.z").as_double());
    if (!camera_model.camera_to_base_translation.allFinite()) {
        throw std::invalid_argument("camera_to_base translation parameters must be finite");
    }
    return camera_model;
}

IcpConfig make_icp_config(rclcpp::Node& node)
{
    IcpConfig config;
    config.max_iterations = node.get_parameter("icp.max_iterations").as_int();
    config.max_correspondence_distance = node.get_parameter("icp.max_correspondence_distance").as_double();
    config.convergence_translation_epsilon =
        node.get_parameter("icp.convergence_translation_epsilon").as_double();
    config.min_correspondences =
        static_cast<std::size_t>(node.get_parameter("icp.min_correspondences").as_int());
    return config;
}

EkfLocalizerConfig make_ekf_config(rclcpp::Node& node)
{
    EkfLocalizerConfig config;
    config.initial_position_variance = node.get_parameter("ekf.initial_position_variance").as_double();
    config.initial_yaw_variance = node.get_parameter("ekf.initial_yaw_variance").as_double();
    config.initial_velocity_variance = node.get_parameter("ekf.initial_velocity_variance").as_double();
    config.initial_yaw_rate_variance = node.get_parameter("ekf.initial_yaw_rate_variance").as_double();
    config.process_position_variance = node.get_parameter("ekf.process_position_variance").as_double();
    config.process_yaw_variance = node.get_parameter("ekf.process_yaw_variance").as_double();
    config.process_velocity_variance = node.get_parameter("ekf.process_velocity_variance").as_double();
    config.process_yaw_rate_variance = node.get_parameter("ekf.process_yaw_rate_variance").as_double();
    config.gnss_position_variance = node.get_parameter("ekf.gnss_position_variance").as_double();
    config.imu_yaw_variance = node.get_parameter("ekf.imu_yaw_variance").as_double();
    config.icp_position_variance = node.get_parameter("ekf.icp_position_variance").as_double();
    return config;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
    const double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(siny_cosp, cosy_cosp);
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

double imu_yaw_to_enu_yaw(const double imu_yaw, const std::string& convention)
{
    if (convention == "heading_north_cw") {
        return normalize_angle(HALF_PI - imu_yaw);
    }
    if (convention == "heading_north_ccw") {
        return normalize_angle(HALF_PI + imu_yaw);
    }
    if (convention == "ros_enu") {
        return normalize_angle(imu_yaw);
    }
    throw std::runtime_error("unsupported imu_yaw_convention: " + convention);
}

void set_yaw(geometry_msgs::msg::Quaternion& quaternion, const double yaw)
{
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
}

cv_bridge::CvImageConstPtr image_to_cv_share(const sensor_msgs::msg::Image::SharedPtr& image_msg)
{
    if (!image_msg) {
        throw std::invalid_argument("image_msg must not be null");
    }
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8 ||
        image_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
    {
        return cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    }
    throw std::invalid_argument("unsupported image encoding: " + image_msg->encoding);
}

}  // namespace

VectormapLocalizationNode::VectormapLocalizationNode(const rclcpp::NodeOptions& options)
: VectormapLocalizationNode("", options)
{
}

VectormapLocalizationNode::VectormapLocalizationNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("vectormap_localization_node", name_space, options),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  map_frame_id_(get_parameter("map_frame_id").as_string()),
  base_frame_id_(get_parameter("base_frame_id").as_string()),
  localized_pose_topic_(get_parameter("localized_pose_topic").as_string()),
  raw_pose_topic_(get_parameter("raw_pose_topic").as_string()),
  velocity_topic_(get_parameter("velocity_topic").as_string()),
  imu_yaw_convention_(get_parameter("imu_yaw_convention").as_string()),
  map_origin_lat_(get_parameter("map_origin_geodetic.latitude").as_double()),
  map_origin_lon_(get_parameter("map_origin_geodetic.longitude").as_double()),
  map_yaw_from_east_(get_parameter("map_yaw_from_east").as_double()),
  mask_threshold_(static_cast<uint8_t>(get_parameter("mask_threshold").as_int())),
  pixel_step_(get_parameter("pixel_step").as_int()),
  max_observed_points_(static_cast<std::size_t>(get_parameter("max_observed_points").as_int())),
  min_observed_points_(static_cast<std::size_t>(get_parameter("min_observed_points").as_int())),
  min_map_points_(static_cast<std::size_t>(get_parameter("min_map_points").as_int())),
  map_sample_interval_m_(get_parameter("map_sample_interval_m").as_double()),
  output_position_variance_(get_parameter("output_position_variance").as_double()),
  output_yaw_variance_(get_parameter("output_yaw_variance").as_double()),
  camera_model_(make_camera_model(*this)),
  icp_matcher_(make_icp_config(*this)),
  ekf_config_(make_ekf_config(*this)),
  ekf_localizer_(ekf_config_),
  qos_(rclcpp::QoS(10)),
  has_last_velocity_update_stamp_(false),
  has_last_gnss_update_stamp_(false),
  has_last_imu_update_stamp_(false)
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }
    if (map_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("map_frame_id and base_frame_id must not be empty");
    }
    if (localized_pose_topic_.empty() || raw_pose_topic_.empty() || velocity_topic_.empty()) {
        throw std::invalid_argument("localization topic parameters must not be empty");
    }
    if (imu_yaw_convention_ != "heading_north_cw" &&
        imu_yaw_convention_ != "heading_north_ccw" &&
        imu_yaw_convention_ != "ros_enu")
    {
        throw std::invalid_argument(
            "imu_yaw_convention must be heading_north_cw, heading_north_ccw, or ros_enu");
    }
    if (!std::isfinite(map_origin_lat_) || !std::isfinite(map_origin_lon_)) {
        throw std::invalid_argument("map_origin_geodetic latitude and longitude must be finite");
    }
    if (!std::isfinite(map_yaw_from_east_)) {
        throw std::invalid_argument("map_yaw_from_east must be finite");
    }
    if (pixel_step_ <= 0) {
        throw std::invalid_argument("pixel_step must be greater than 0");
    }
    if (max_observed_points_ == 0U || min_observed_points_ == 0U || min_map_points_ == 0U) {
        throw std::invalid_argument("point count parameters must be greater than 0");
    }
    if (map_sample_interval_m_ <= 0.0) {
        throw std::invalid_argument("map_sample_interval_m must be greater than 0");
    }
    if (output_position_variance_ <= 0.0 || output_yaw_variance_ <= 0.0) {
        throw std::invalid_argument("output covariance parameters must be greater than 0");
    }

    mask_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolop/mask_image",
        qos_,
        std::bind(&VectormapLocalizationNode::mask_callback, this, std::placeholders::_1));
    vector_map_subscription_ = this->create_subscription<vectormap_msgs::msg::VectorMap>(
        "/vector_map",
        qos_,
        std::bind(&VectormapLocalizationNode::vector_map_callback, this, std::placeholders::_1));
    gnss_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/vectornav/gnss",
        qos_,
        std::bind(&VectormapLocalizationNode::gnss_callback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/vectornav/imu",
        qos_,
        std::bind(&VectormapLocalizationNode::imu_callback, this, std::placeholders::_1));
    velocity_subscription_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            velocity_topic_,
            qos_,
            std::bind(&VectormapLocalizationNode::velocity_callback, this, std::placeholders::_1));

    localized_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        localized_pose_topic_,
        qos_);
    raw_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        raw_pose_topic_,
        qos_);
    lane_line_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/detection/lane_line", qos_);
    lane_line_map_raw_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detection/lane_line_map_raw", qos_);
    lane_line_map_corrected_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detection/lane_line_map_corrected", qos_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&VectormapLocalizationNode::timer_callback, this));
}

void VectormapLocalizationNode::mask_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_mask_image_ = msg;
}

void VectormapLocalizationNode::vector_map_callback(const vectormap_msgs::msg::VectorMap::SharedPtr msg)
{
    rebuild_map_points(*msg);
}

void VectormapLocalizationNode::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_gnss_msg_ = msg;
}

void VectormapLocalizationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_msg_ = msg;
}

void VectormapLocalizationNode::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if (!msg) {
        throw std::runtime_error("velocity message must not be null");
    }
    if (!is_valid_velocity_frame(msg->header.frame_id)) {
        throw std::runtime_error(
            "velocity_body frame_id must be " + base_frame_id_ + " or vectornav, got " +
            msg->header.frame_id);
    }
    if (!std::isfinite(msg->twist.twist.linear.x) ||
        !std::isfinite(msg->twist.twist.linear.y) ||
        !std::isfinite(msg->twist.twist.angular.z))
    {
        throw std::runtime_error("velocity_body twist contains non-finite values");
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_msg_ = msg;
}

void VectormapLocalizationNode::rebuild_map_points(const vectormap_msgs::msg::VectorMap& map_msg)
{
    if (map_msg.header.frame_id != map_frame_id_) {
        throw std::runtime_error(
            "VectorMap frame_id must be " + map_frame_id_ + ", got " + map_msg.header.frame_id);
    }

    std::vector<Eigen::Vector2d> points;
    for (const auto& line_string : map_msg.line_strings) {
        if (!line_string.is_observable || line_string.marking_type == LineString::MARKING_VIRTUAL) {
            continue;
        }
        if (line_string.points.size() < 2U) {
            throw std::runtime_error("observable LineString must contain at least two points");
        }

        for (std::size_t i = 1U; i < line_string.points.size(); ++i) {
            const Eigen::Vector2d start(line_string.points[i - 1U].x, line_string.points[i - 1U].y);
            const Eigen::Vector2d end(line_string.points[i].x, line_string.points[i].y);
            const Eigen::Vector2d delta = end - start;
            const double length = delta.norm();
            const int samples = std::max(1, static_cast<int>(std::ceil(length / map_sample_interval_m_)));
            for (int sample = 0; sample <= samples; ++sample) {
                const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
                points.push_back(start + ratio * delta);
            }
        }
    }

    if (points.size() < min_map_points_) {
        throw std::runtime_error("not enough observable vector map points for localization");
    }
    auto target_map = std::make_shared<IcpTargetMap>(std::move(points));
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_points_ = target_map;
    RCLCPP_INFO(this->get_logger(), "rebuilt localization map points: %zu", target_map->size());
}

bool VectormapLocalizationNode::gnss_to_map_pose(
    const sensor_msgs::msg::NavSatFix& gnss_msg,
    const sensor_msgs::msg::Imu& imu_msg,
    geometry_msgs::msg::PoseWithCovarianceStamped& pose_out) const
{
    if (gnss_msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        return false;
    }
    if (!std::isfinite(gnss_msg.latitude) || !std::isfinite(gnss_msg.longitude)) {
        throw std::runtime_error("GNSS latitude or longitude is not finite");
    }

    // lat/lon → ENU ローカル接平面
    // 子午線曲率半径 M と卯酉線曲率半径 N で緯度経度差をメートルに変換
    const double lat0_rad = map_origin_lat_ * DEG2RAD;
    const double sin_lat0 = std::sin(lat0_rad);
    const double sin2_lat0 = sin_lat0 * sin_lat0;
    const double denom = std::sqrt(1.0 - WGS84_E2 * sin2_lat0);
    const double N = WGS84_A / denom;
    const double M = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);

    const double delta_lat = (gnss_msg.latitude - map_origin_lat_) * DEG2RAD;
    const double delta_lon = (gnss_msg.longitude - map_origin_lon_) * DEG2RAD;
    const double north = M * delta_lat;
    const double east = N * std::cos(lat0_rad) * delta_lon;

    // ENU → map: [x_map, y_map]^T = R_map_enu * [east, north]^T
    const double cos_yaw = std::cos(map_yaw_from_east_);
    const double sin_yaw = std::sin(map_yaw_from_east_);
    const double x_map = cos_yaw * east + sin_yaw * north;
    const double y_map = -sin_yaw * east + cos_yaw * north;

    const double yaw_enu = imu_yaw_to_enu_yaw(
        yaw_from_quaternion(imu_msg.orientation),
        imu_yaw_convention_);
    const double yaw_map = normalize_angle(yaw_enu - map_yaw_from_east_);

    pose_out.header.stamp = gnss_msg.header.stamp;
    pose_out.header.frame_id = map_frame_id_;
    pose_out.pose.pose.position.x = x_map;
    pose_out.pose.pose.position.y = y_map;
    pose_out.pose.pose.position.z = 0.0;
    set_yaw(pose_out.pose.pose.orientation, yaw_map);
    pose_out.pose.covariance.fill(0.0);

    return true;
}

geometry_msgs::msg::PoseWithCovarianceStamped VectormapLocalizationNode::update_ekf_with_raw_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& raw_pose,
    const sensor_msgs::msg::Imu& imu_msg,
    const geometry_msgs::msg::TwistWithCovarianceStamped& velocity_msg)
{
    if (!is_valid_velocity_frame(velocity_msg.header.frame_id)) {
        throw std::runtime_error(
            "velocity_body frame_id must be " + base_frame_id_ + " or vectornav, got " +
            velocity_msg.header.frame_id);
    }
    if (!std::isfinite(velocity_msg.twist.twist.linear.x) ||
        !std::isfinite(velocity_msg.twist.twist.angular.z))
    {
        throw std::runtime_error("velocity_body twist contains non-finite values");
    }
    if (!std::isfinite(imu_msg.angular_velocity.z)) {
        throw std::runtime_error("IMU angular_velocity.z is not finite");
    }

    const rclcpp::Time velocity_stamp(velocity_msg.header.stamp, get_clock()->get_clock_type());
    const double raw_x = raw_pose.pose.pose.position.x;
    const double raw_y = raw_pose.pose.pose.position.y;
    const double raw_yaw = yaw_from_quaternion(raw_pose.pose.pose.orientation);
    const double velocity = velocity_msg.twist.twist.linear.x;
    const double yaw_rate = imu_msg.angular_velocity.z;

    if (!ekf_localizer_.initialized()) {
        ekf_localizer_.initialize(raw_x, raw_y, raw_yaw, velocity, yaw_rate, velocity_stamp);
        has_last_velocity_update_stamp_ = true;
        last_velocity_update_stamp_ = velocity_msg.header.stamp;
        has_last_gnss_update_stamp_ = true;
        last_gnss_update_stamp_ = raw_pose.header.stamp;
        has_last_imu_update_stamp_ = true;
        last_imu_update_stamp_ = imu_msg.header.stamp;
        return ekf_localizer_.make_pose(this->now(), map_frame_id_);
    } else if (
        !has_last_velocity_update_stamp_ ||
        !same_stamp(velocity_msg.header.stamp, last_velocity_update_stamp_))
    {
        ekf_localizer_.predict(velocity, yaw_rate, velocity_stamp);
        has_last_velocity_update_stamp_ = true;
        last_velocity_update_stamp_ = velocity_msg.header.stamp;
    }

    if (!has_last_gnss_update_stamp_ || !same_stamp(raw_pose.header.stamp, last_gnss_update_stamp_)) {
        ekf_localizer_.update_position(raw_x, raw_y, ekf_config_.gnss_position_variance);
        has_last_gnss_update_stamp_ = true;
        last_gnss_update_stamp_ = raw_pose.header.stamp;
    }

    if (!has_last_imu_update_stamp_ || !same_stamp(imu_msg.header.stamp, last_imu_update_stamp_)) {
        ekf_localizer_.update_yaw(raw_yaw, ekf_config_.imu_yaw_variance);
        has_last_imu_update_stamp_ = true;
        last_imu_update_stamp_ = imu_msg.header.stamp;
    }

    return ekf_localizer_.make_pose(this->now(), map_frame_id_);
}

bool VectormapLocalizationNode::is_valid_velocity_frame(const std::string& frame_id) const
{
    return frame_id == base_frame_id_ || frame_id == "vectornav";
}

bool VectormapLocalizationNode::same_stamp(
    const builtin_interfaces::msg::Time& lhs,
    const builtin_interfaces::msg::Time& rhs)
{
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

std::vector<Eigen::Vector2d> VectormapLocalizationNode::observed_points_in_initial_map(
    const std::vector<Eigen::Vector2d>& base_points,
    const geometry_msgs::msg::PoseWithCovarianceStamped& initial_pose) const
{
    const double yaw = yaw_from_quaternion(initial_pose.pose.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const Eigen::Vector2d translation(
        initial_pose.pose.pose.position.x,
        initial_pose.pose.pose.position.y);

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

geometry_msgs::msg::PoseWithCovarianceStamped VectormapLocalizationNode::make_localized_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& initial_pose,
    const Eigen::Vector2d& correction) const
{
    geometry_msgs::msg::PoseWithCovarianceStamped localized_pose = initial_pose;
    localized_pose.header.stamp = this->now();
    localized_pose.header.frame_id = map_frame_id_;
    localized_pose.pose.pose.position.x += correction.x();
    localized_pose.pose.pose.position.y += correction.y();
    localized_pose.pose.pose.position.z = 0.0;
    set_yaw(localized_pose.pose.pose.orientation, yaw_from_quaternion(initial_pose.pose.pose.orientation));
    localized_pose.pose.covariance.fill(0.0);
    localized_pose.pose.covariance[0] = output_position_variance_;
    localized_pose.pose.covariance[7] = output_position_variance_;
    localized_pose.pose.covariance[35] = output_yaw_variance_;
    return localized_pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped VectormapLocalizationNode::update_ekf_with_icp_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& icp_pose)
{
    ekf_localizer_.update_position(
        icp_pose.pose.pose.position.x,
        icp_pose.pose.pose.position.y,
        ekf_config_.icp_position_variance);
    return ekf_localizer_.make_pose(this->now(), map_frame_id_);
}

visualization_msgs::msg::MarkerArray VectormapLocalizationNode::make_lane_line_marker_array(
    const std::vector<Eigen::Vector2d>& base_points) const
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = this->now();
    delete_marker.header.frame_id = base_frame_id_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker points_marker;
    points_marker.header = delete_marker.header;
    points_marker.ns = "lane_line_base_link";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::POINTS;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.08;
    points_marker.scale.y = 0.08;
    points_marker.color.r = 0.0F;
    points_marker.color.g = 1.0F;
    points_marker.color.b = 0.2F;
    points_marker.color.a = 1.0F;
    points_marker.points.reserve(base_points.size());

    for (const auto& base_point : base_points) {
        geometry_msgs::msg::Point point;
        point.x = base_point.x();
        point.y = base_point.y();
        point.z = camera_model_.ground_plane_z_base;
        points_marker.points.push_back(point);
    }

    marker_array.markers.push_back(std::move(points_marker));
    return marker_array;
}

visualization_msgs::msg::MarkerArray VectormapLocalizationNode::make_lane_line_map_marker_array(
    const std::vector<Eigen::Vector2d>& map_points,
    const std::string& ns,
    const float r, const float g, const float b) const
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = this->now();
    delete_marker.header.frame_id = map_frame_id_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker points_marker;
    points_marker.header = delete_marker.header;
    points_marker.ns = ns;
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::POINTS;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.08;
    points_marker.scale.y = 0.08;
    points_marker.color.r = r;
    points_marker.color.g = g;
    points_marker.color.b = b;
    points_marker.color.a = 1.0F;
    points_marker.points.reserve(map_points.size());

    for (const auto& p : map_points) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        points_marker.points.push_back(point);
    }

    marker_array.markers.push_back(std::move(points_marker));
    return marker_array;
}

void VectormapLocalizationNode::timer_callback()
{
    sensor_msgs::msg::Image::SharedPtr mask_image_msg;
    sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr velocity_msg;
    std::shared_ptr<const IcpTargetMap> map_points;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        mask_image_msg = latest_mask_image_;
        gnss_msg = latest_gnss_msg_;
        imu_msg = latest_imu_msg_;
        velocity_msg = latest_velocity_msg_;
        map_points = map_points_;
    }

    if (!gnss_msg || !imu_msg || !velocity_msg) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "waiting for GNSS, IMU, and velocity_body");
        return;
    }

    try {
        geometry_msgs::msg::PoseWithCovarianceStamped raw_pose;
        if (!gnss_to_map_pose(*gnss_msg, *imu_msg, raw_pose)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "GNSS fix not available, skipping localization");
            return;
        }
        raw_pose_publisher_->publish(raw_pose);

        const auto initial_pose = update_ekf_with_raw_pose(raw_pose, *imu_msg, *velocity_msg);

        if (!mask_image_msg || !map_points || map_points->empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "waiting for mask image and vector map");
            localized_pose_publisher_->publish(initial_pose);
            return;
        }

        const auto mask_image = image_to_cv_share(mask_image_msg);
        cv::Mat binary_mask;
        cv::threshold(mask_image->image, binary_mask, mask_threshold_, 255.0, cv::THRESH_BINARY);

        cv::Mat skeleton_mask;
        cv::ximgproc::thinning(binary_mask, skeleton_mask, cv::ximgproc::THINNING_ZHANGSUEN);

        if (ground_projection_lut_.empty() ||
            ground_projection_lut_.image_width != skeleton_mask.cols ||
            ground_projection_lut_.image_height != skeleton_mask.rows)
        {
            ground_projection_lut_ = build_ground_projection_lut(
                camera_model_,
                pixel_step_,
                skeleton_mask.cols,
                skeleton_mask.rows);
            RCLCPP_INFO(
                this->get_logger(),
                "rebuilt ground projection LUT: image=%dx%d, entries=%zu",
                skeleton_mask.cols,
                skeleton_mask.rows,
                ground_projection_lut_.size());
        }

        const auto base_points = lane_pixels_to_base_points(
            skeleton_mask,
            ground_projection_lut_,
            mask_threshold_,
            max_observed_points_);
        lane_line_publisher_->publish(make_lane_line_marker_array(base_points));
        const auto raw_source_points = observed_points_in_initial_map(base_points, raw_pose);
        lane_line_map_raw_publisher_->publish(
            make_lane_line_map_marker_array(raw_source_points, "lane_line_map_raw", 1.0F, 1.0F, 0.0F));
        if (base_points.size() < min_observed_points_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "not enough observed lane points: %zu",
                base_points.size());
            localized_pose_publisher_->publish(initial_pose);
            return;
        }

        const auto source_points = observed_points_in_initial_map(base_points, initial_pose);

        const auto result = icp_matcher_.align_translation_only(source_points, *map_points);
        if (!result.converged) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "ICP failed: correspondences=%zu, mean_error=%.3f",
                result.correspondences,
                result.mean_error);
            localized_pose_publisher_->publish(initial_pose);
            return;
        }

        std::vector<Eigen::Vector2d> corrected_points;
        corrected_points.reserve(source_points.size());
        for (const auto& p : source_points) {
            corrected_points.push_back(p + result.translation);
        }
        lane_line_map_corrected_publisher_->publish(
            make_lane_line_map_marker_array(
                corrected_points, "lane_line_map_corrected", 0.0F, 1.0F, 1.0F));

        const auto icp_pose = make_localized_pose(initial_pose, result.translation);
        localized_pose_publisher_->publish(update_ekf_with_icp_pose(icp_pose));
        RCLCPP_DEBUG(
            this->get_logger(),
            "localized correction=(%.3f, %.3f), correspondences=%zu, mean_error=%.3f",
            result.translation.x(),
            result.translation.y(),
            result.correspondences,
            result.mean_error);
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "localization skipped: %s",
            error.what());
    }
}

}  // namespace vectormap_matching
