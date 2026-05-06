#include "road_segments_provider/road_segments_provider_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <mapless_planning_msgs/msg/linestring.hpp>
#include <mapless_planning_msgs/msg/segment.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace road_segments_provider
{

RoadSegmentsProviderNode::RoadSegmentsProviderNode(const rclcpp::NodeOptions & options)
: RoadSegmentsProviderNode("", options)
{
}

RoadSegmentsProviderNode::RoadSegmentsProviderNode(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: Node("road_segments_provider_node", name_space, options),
    camera_info_topic_(get_parameter("camera_info_topic").as_string()),
    lane_mask_topic_(get_parameter("lane_mask_topic").as_string()),
    da_mask_topic_(get_parameter("da_mask_topic").as_string()),
    road_segments_topic_(get_parameter("road_segments_topic").as_string()),
    camera_frame_id_(get_parameter("camera_frame_id").as_string()),
    base_frame_id_(get_parameter("base_frame_id").as_string()),
    min_ground_x_m_(get_parameter("min_ground_x_m").as_double()),
    max_ground_x_m_(get_parameter("max_ground_x_m").as_double()),
    lane_width_m_(get_parameter("lane_width_m").as_double()),
    min_valid_pixel_width_(get_parameter("min_valid_pixel_width").as_int()),
    poly_fit_order_(get_parameter("poly_fit_order").as_int()),
    min_fit_points_(get_parameter("min_fit_points").as_int()),
    centerline_resample_interval_m_(get_parameter("centerline_resample_interval_m").as_double()),
    publish_markers_(get_parameter("publish_markers").as_bool()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
{
    if (min_ground_x_m_ < 0.0 || max_ground_x_m_ <= min_ground_x_m_) {
        throw std::invalid_argument(
            "Invalid ground range: min_ground_x_m must be >= 0 and < max_ground_x_m");
    }
    if (lane_width_m_ <= 0.0) {
        throw std::invalid_argument("lane_width_m must be greater than 0");
    }
    if (poly_fit_order_ < 2 || poly_fit_order_ > 3) {
        throw std::invalid_argument("poly_fit_order must be 2 or 3");
    }
    if (min_fit_points_ < poly_fit_order_ + 1) {
        throw std::invalid_argument("min_fit_points must be > poly_fit_order");
    }
    if (centerline_resample_interval_m_ <= 0.0) {
        throw std::invalid_argument("centerline_resample_interval_m must be > 0");
    }

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::QoS(10),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
            camera_info_callback(msg);
        });

    lane_mask_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, lane_mask_topic_, rclcpp::QoS(10).get_rmw_qos_profile());
    da_mask_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, da_mask_topic_, rclcpp::QoS(10).get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *lane_mask_sub_, *da_mask_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
    sync_->registerCallback(
        std::bind(
            &RoadSegmentsProviderNode::sync_callback, this,
            std::placeholders::_1, std::placeholders::_2));

    road_segments_pub_ = create_publisher<mapless_planning_msgs::msg::RoadSegments>(
        road_segments_topic_, rclcpp::QoS(10));

    if (publish_markers_) {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            road_segments_topic_ + "/markers", rclcpp::QoS(10));
    }

    RCLCPP_INFO(get_logger(), "RoadSegmentsProviderNode initialized");
    RCLCPP_INFO(get_logger(), "  lane_mask: %s", lane_mask_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  da_mask:   %s", da_mask_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output:    %s", road_segments_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  poly_fit_order: %d, min_fit_points: %d, resample: %.2f m",
        poly_fit_order_, min_fit_points_, centerline_resample_interval_m_);
}

void RoadSegmentsProviderNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (camera_intrinsics_) {
        return;  // already set
    }
    // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1] (row-major)
    camera_intrinsics_ = {msg->k[0], msg->k[4], msg->k[2], msg->k[5]};
    RCLCPP_INFO(get_logger(), "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
        (*camera_intrinsics_)[0], (*camera_intrinsics_)[1],
        (*camera_intrinsics_)[2], (*camera_intrinsics_)[3]);
}

std::optional<CameraParams> RoadSegmentsProviderNode::build_camera_params()
{
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (!camera_intrinsics_) {
        return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_.lookupTransform(
            base_frame_id_, camera_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF lookup %s -> %s failed: %s",
            camera_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
        return std::nullopt;
    }

    const auto & q = tf_stamped.transform.rotation;
    const auto & tr = tf_stamped.transform.translation;

    CameraParams params;
    params.fx = (*camera_intrinsics_)[0];
    params.fy = (*camera_intrinsics_)[1];
    params.cx = (*camera_intrinsics_)[2];
    params.cy = (*camera_intrinsics_)[3];
    params.R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w);
    params.t = {tr.x, tr.y, tr.z};
    return params;
}

std::pair<std::vector<double>, std::vector<double>>
RoadSegmentsProviderNode::collect_center_points(
    const cv::Mat & lane_img,
    const CameraParams & params) const
{
    std::vector<double> cx_out, cy_out;

    const int img_width = lane_img.cols;
    const int height = lane_img.rows;
    const int img_cx = img_width / 2;

    for (int v = height - 1; v >= 0; --v) {
        const uchar * row = lane_img.ptr<uchar>(v);

        int left_u = -1;
        int right_u = -1;

        for (int u = img_cx - 1; u >= 0; --u) {
            if (row[u] != 0) { left_u = u; break; }
        }
        for (int u = img_cx; u < img_width; ++u) {
            if (row[u] != 0) { right_u = u; break; }
        }

        if (left_u < 0 || right_u < 0) {
            continue;
        }
        if (right_u - left_u < min_valid_pixel_width_) {
            continue;
        }

        const int u_center = (left_u + right_u) / 2;
        auto pt = project_pixel_to_ground(u_center, v, params);
        if (!pt || pt->x < min_ground_x_m_ || pt->x > max_ground_x_m_) {
            continue;
        }

        cx_out.push_back(pt->x);
        cy_out.push_back(pt->y);
    }

    return {cx_out, cy_out};
}

LaneBoundaries RoadSegmentsProviderNode::fit_and_expand(
    const std::vector<double> & cx,
    const std::vector<double> & cy) const
{
    const int N = static_cast<int>(cx.size());
    if (N < min_fit_points_) {
        RCLCPP_DEBUG(get_logger(), "Too few center points (%d < %d), skipping segment", N, min_fit_points_);
        return {};
    }

    // Build design matrix A (N × order+1) and RHS b (N × 1)
    const int cols = poly_fit_order_ + 1;
    cv::Mat A(N, cols, CV_64F);
    cv::Mat b(N, 1, CV_64F);
    for (int i = 0; i < N; ++i) {
        double xi = 1.0;
        for (int j = 0; j < cols; ++j) {
            A.at<double>(i, j) = xi;
            xi *= cx[i];
        }
        b.at<double>(i, 0) = cy[i];
    }

    cv::Mat coeff;
    if (!cv::solve(A, b, coeff, cv::DECOMP_SVD)) {
        RCLCPP_WARN(get_logger(), "Polynomial fit failed (cv::solve returned false)");
        return {};
    }

    // Resample fitted centerline at uniform x intervals
    const auto [x_min_it, x_max_it] = std::minmax_element(cx.begin(), cx.end());
    const double x_min = *x_min_it;
    const double x_max = *x_max_it;
    const int n_samples = std::max(2,
        static_cast<int>((x_max - x_min) / centerline_resample_interval_m_) + 1);

    const double half_width = lane_width_m_ * 0.5;
    LaneBoundaries result;
    result.left.reserve(n_samples);
    result.right.reserve(n_samples);

    for (int i = 0; i < n_samples; ++i) {
        const double x = x_min + (x_max - x_min) * i / (n_samples - 1);
        double y = 0.0;
        double xi = 1.0;
        for (int j = 0; j < cols; ++j) {
            y += coeff.at<double>(j, 0) * xi;
            xi *= x;
        }
        geometry_msgs::msg::Point lp, rp;
        lp.x = rp.x = x;
        lp.z = rp.z = 0.0;
        lp.y = y + half_width;
        rp.y = y - half_width;
        result.left.push_back(lp);
        result.right.push_back(rp);
    }

    // κ(x) ≈ 2c₂ + 6c₃x; evaluate at near and far ends, take the larger magnitude
    const double c2 = coeff.at<double>(2, 0);
    const double c3 = (poly_fit_order_ >= 3) ? coeff.at<double>(3, 0) : 0.0;
    const double kappa_near = 2.0 * c2 + 6.0 * c3 * x_min;
    const double kappa_far  = 2.0 * c2 + 6.0 * c3 * x_max;
    result.curvature = static_cast<float>(
        std::abs(kappa_near) >= std::abs(kappa_far) ? kappa_near : kappa_far);

    return result;
}

visualization_msgs::msg::MarkerArray RoadSegmentsProviderNode::build_markers(
    const std::vector<geometry_msgs::msg::Point> & left,
    const std::vector<geometry_msgs::msg::Point> & right,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
{
    visualization_msgs::msg::MarkerArray arr;

    auto make_line_marker = [&](
        const std::vector<geometry_msgs::msg::Point> & pts,
        int id, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = "lane_boundary";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;
        m.points = pts;
        return m;
    };

    if (!left.empty()) {
        arr.markers.push_back(make_line_marker(left, 0, 1.0f, 1.0f, 0.0f));
    }
    if (!right.empty()) {
        arr.markers.push_back(make_line_marker(right, 1, 0.0f, 1.0f, 1.0f));
    }
    return arr;
}

void RoadSegmentsProviderNode::sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & lane_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & /*da_msg*/)
{
    const auto params_opt = build_camera_params();
    if (!params_opt) {
        return;
    }
    const auto & params = *params_opt;

    cv_bridge::CvImageConstPtr lane_cv;
    try {
        lane_cv = cv_bridge::toCvShare(lane_msg, "mono8");
    } catch (const cv_bridge::Exception & ex) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", ex.what());
        return;
    }

    const cv::Mat & lane_img = lane_cv->image;
    if (lane_img.empty()) {
        return;
    }

    auto [cx, cy] = collect_center_points(lane_img, params);
    const LaneBoundaries boundaries = fit_and_expand(cx, cy);

    if (boundaries.left.empty()) {
        RCLCPP_DEBUG(get_logger(), "No valid lane boundaries after polynomial fit");
        return;
    }

    // Build ego segment
    mapless_planning_msgs::msg::Segment segment;
    segment.id = 0;
    segment.curvature = boundaries.curvature;

    for (const auto & pt : boundaries.left) {
        geometry_msgs::msg::Pose pose;
        pose.position = pt;
        pose.orientation.w = 1.0;
        segment.linestrings[0].poses.push_back(pose);
    }
    for (const auto & pt : boundaries.right) {
        geometry_msgs::msg::Pose pose;
        pose.position = pt;
        pose.orientation.w = 1.0;
        segment.linestrings[1].poses.push_back(pose);
    }

    mapless_planning_msgs::msg::RoadSegments msg;
    msg.header.stamp = lane_msg->header.stamp;
    msg.header.frame_id = base_frame_id_;
    msg.segments.push_back(segment);
    msg.pose.orientation.w = 1.0;

    road_segments_pub_->publish(msg);

    if (publish_markers_ && marker_pub_) {
        marker_pub_->publish(
            build_markers(boundaries.left, boundaries.right,
                base_frame_id_, lane_msg->header.stamp));
    }
}

}  // namespace road_segments_provider
