#include "road_segments_provider/road_segments_provider_node.hpp"

#include <algorithm>
#include <stdexcept>

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
    camera_info_topic_(declare_parameter<std::string>(
        "camera_info_topic", "/zed/zed_node/rgb/camera_info")),
    lane_mask_topic_(
        declare_parameter<std::string>("lane_mask_topic", "/perception/lane_mask")),
    da_mask_topic_(
        declare_parameter<std::string>("da_mask_topic", "/perception/drivable_area_mask")),
    road_segments_topic_(
        declare_parameter<std::string>("road_segments_topic", "/perception/road_segments")),
    camera_frame_id_(
        declare_parameter<std::string>("camera_frame_id", "camera_link")),
    base_frame_id_(
        declare_parameter<std::string>("base_frame_id", "base_link")),
    min_ground_x_m_(declare_parameter<double>("min_ground_x_m", 0.5)),
    max_ground_x_m_(declare_parameter<double>("max_ground_x_m", 20.0)),
    publish_markers_(declare_parameter<bool>("publish_markers", false)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
{
    if (min_ground_x_m_ < 0.0 || max_ground_x_m_ <= min_ground_x_m_) {
        throw std::invalid_argument(
            "Invalid ground range: min_ground_x_m must be >= 0 and < max_ground_x_m");
    }

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::QoS(1).transient_local(),
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

std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>>
RoadSegmentsProviderNode::extract_lane_boundaries(
    const cv::Mat & lane_img,
    const CameraParams & params) const
{
    std::vector<geometry_msgs::msg::Point> left_pts;
    std::vector<geometry_msgs::msg::Point> right_pts;

    const int width = lane_img.cols;
    const int height = lane_img.rows;
    const int cx = width / 2;

    // Scan rows from bottom (nearest) to top (farthest) to build ordered boundaries
    for (int v = height - 1; v >= 0; --v) {
        const uchar * row = lane_img.ptr<uchar>(v);

        int left_u = -1;   // rightmost non-zero pixel in left half
        int right_u = -1;  // leftmost non-zero pixel in right half

        for (int u = cx - 1; u >= 0; --u) {
            if (row[u] != 0) {
                left_u = u;
                break;
            }
        }
        for (int u = cx; u < width; ++u) {
            if (row[u] != 0) {
                right_u = u;
                break;
            }
        }

        if (left_u >= 0) {
            auto pt = project_pixel_to_ground(left_u, v, params);
            if (pt && pt->x >= min_ground_x_m_ && pt->x <= max_ground_x_m_) {
                left_pts.push_back(*pt);
            }
        }
        if (right_u >= 0) {
            auto pt = project_pixel_to_ground(right_u, v, params);
            if (pt && pt->x >= min_ground_x_m_ && pt->x <= max_ground_x_m_) {
                right_pts.push_back(*pt);
            }
        }
    }

    // Already ordered near to far (bottom row projected first, smallest x first in base_link)
    // Sort by x ascending just in case projection re-orders due to camera tilt
    auto by_x = [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
        return a.x < b.x;
    };
    std::sort(left_pts.begin(), left_pts.end(), by_x);
    std::sort(right_pts.begin(), right_pts.end(), by_x);

    return {left_pts, right_pts};
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

    auto [left_pts, right_pts] = extract_lane_boundaries(lane_img, params);

    if (left_pts.empty() && right_pts.empty()) {
        RCLCPP_DEBUG(get_logger(), "No lane boundaries detected");
        return;
    }

    // Build ego segment
    mapless_planning_msgs::msg::Segment segment;
    segment.id = 0;

    // linestrings[0] = left boundary
    for (const auto & pt : left_pts) {
        geometry_msgs::msg::Pose pose;
        pose.position = pt;
        pose.orientation.w = 1.0;
        segment.linestrings[0].poses.push_back(pose);
    }
    // linestrings[1] = right boundary
    for (const auto & pt : right_pts) {
        geometry_msgs::msg::Pose pose;
        pose.position = pt;
        pose.orientation.w = 1.0;
        segment.linestrings[1].poses.push_back(pose);
    }

    mapless_planning_msgs::msg::RoadSegments msg;
    msg.header.stamp = lane_msg->header.stamp;
    msg.header.frame_id = base_frame_id_;
    msg.segments.push_back(segment);
    // pose = identity: segments are expressed in base_link frame, ego is at origin
    msg.pose.orientation.w = 1.0;

    road_segments_pub_->publish(msg);

    if (publish_markers_ && marker_pub_) {
        marker_pub_->publish(
            build_markers(left_pts, right_pts, base_frame_id_, lane_msg->header.stamp));
    }
}

}  // namespace road_segments_provider
