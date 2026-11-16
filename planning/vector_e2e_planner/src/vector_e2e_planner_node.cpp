#include "vector_e2e_planner/vector_e2e_planner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <utilities/utils.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <utility>

namespace vector_e2e_planner {

namespace {

constexpr std::size_t min_output_points = 3;  // speed_path_planner の曲率計算が 3 点を要求する

// course は /** の共通パラメータ。blue なら true
bool is_blue_course(const std::string& course) {
    if (course != "blue" && course != "red") {
        throw std::invalid_argument("course は \"blue\" か \"red\" を指定してください: " + course);
    }
    return course == "blue";
}

}  // namespace

VectorE2ePlannerNode::VectorE2ePlannerNode(const rclcpp::NodeOptions& options)
    : VectorE2ePlannerNode("", options) {
}

VectorE2ePlannerNode::VectorE2ePlannerNode(
    const std::string&         name_space,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("vector_e2e_planner_node", name_space, options),
      blue_(is_blue_course(get_parameter("course").as_string())),
      signal_red_threshold_(get_parameter("signal_red_threshold").as_double()),
      stop_offset_m_(get_parameter("stop_offset_m").as_double()),
      path_valid_threshold_(get_parameter("path_valid_threshold").as_double()),
      output_interval_m_(get_parameter("output_interval_m").as_double()),
      tf_timeout_s_(get_parameter("tf_timeout_s").as_double()),
      route_plan_(get_parameter("route_plan").as_string_array()),
      counter_(),
      smoother_(get_parameter("path_ema_alpha").as_double()),
      base_T_camera_(camera_utility::getBaseTCamera(*this)),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
    const std::string engine_path =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("vector_e2e_planner")) /
        "weights" / get_parameter("engine_path").as_string();
    inference_ = std::make_unique<VectorE2eTensorrt>(engine_path);
    RCLCPP_INFO(get_logger(), "vector_plannerエンジンを読み込みました: %s", engine_path.c_str());

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>("/zed/zed_node/rgb/camera_info", rclcpp::SystemDefaultsQoS(), std::bind(&VectorE2ePlannerNode::camera_info_callback, this, std::placeholders::_1));
    image_subscription_       = create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", rclcpp::SensorDataQoS().keep_last(1), std::bind(&VectorE2ePlannerNode::image_callback, this, std::placeholders::_1));

    local_path_publisher_  = create_publisher<nav_msgs::msg::Path>("/planner/local_path", rclcpp::QoS(1));
    stop_point_publisher_  = create_publisher<speed_path_msgs::msg::StopPoint>("/planning/stop_point", rclcpp::QoS(1));
    marker_publisher_      = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/vector_e2e_visualize", rclcpp::QoS(1));
    debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/planning/vector_e2e_debug_image", rclcpp::SensorDataQoS().keep_last(1));
}

void VectorE2ePlannerNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (intrinsics_) return;

    intrinsics_ = camera_utility::fromCameraInfo(*msg);
    RCLCPP_INFO(get_logger(), "camera_info(%ux%u)を受信しました", msg->width, msg->height);
}

std::optional<Pose2D> VectorE2ePlannerNode::lookup_odom_pose(const builtin_interfaces::msg::Time& stamp) {
    try {
        const auto transform = tf_buffer_.lookupTransform(
            "odom", "base_link", tf2_ros::fromMsg(stamp), tf2::durationFromSec(tf_timeout_s_));
        return Pose2D{
            transform.transform.translation.x, transform.transform.translation.y,
            utils::yaw_from_quaternion(transform.transform.rotation)};
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "odom -> base_link のTFが引けないためスキップします: %s", error.what());
        return std::nullopt;
    }
}

void VectorE2ePlannerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const std::optional<Pose2D> pose = lookup_odom_pose(msg->header.stamp);
    if (!pose) return;

    // 学習は cv2.imread の BGR をそのまま入力している
    const cv::Mat source = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    // letterbox処理 360x640 → 384x640
    cv::Mat padded;
    cv::copyMakeBorder(
        source, padded, pad_top, input_height - source_height - pad_top,
        pad_left, input_width - source_width - pad_left, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    const ModelOutputs outputs = inference_->infer(padded);
    if (!outputs.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "推論に失敗しました: %s", inference_->last_error().c_str());
        return;
    }

    // 走行距離は odom 上の移動量の積算。最初のフレームは 0
    const double travel_m = previous_pose_ ? std::hypot(pose->x - previous_pose_->x, pose->y - previous_pose_->y) : 0.0;
    previous_pose_        = pose;

    // 青コースだけ分岐を数えて route_plan で経路を選ぶ。赤コースは常に straight
    Slot commanded = Slot::Straight;
    if (blue_) {
        const std::optional<BranchEvent> event =
            counter_.update(outputs.branch_exist, outputs.corridor_left, outputs.corridor_right, outputs.path_valid, travel_m);
        commanded = route_plan_.next_slot(counter_.count());
        if (event) {
            RCLCPP_INFO(
                get_logger(), "分岐を通過しました #%d kind=%s reason=%s travel=%.1fm 次の指示=%s", event->index + 1,
                to_string(event->kind), to_string(event->reason), event->travel_m, slot_name(commanded));
        }
    }
    const Slot slot      = select_slot(commanded, outputs.branch_exist, exist_threshold_);
    const std::size_t index = static_cast<std::size_t>(slot);

    // EMA は 40 点の全体にかけ、path_valid による打ち切りは平滑後に行う
    const auto smoothed = smoother_.step(outputs.path_xy[index], slot, *pose);
    const auto points   = build_output_points(valid_prefix(smoothed, outputs.path_valid[index], path_valid_threshold_), output_interval_m_);

    if (points.size() >= min_output_points) {
        // 信号停止。停止点は出力経路先頭（自車位置）からの弧長で、speed_path_planner が経路先頭を 0 として解釈する
        if (!blue_ && outputs.signal_prob[signal_red_index] >= signal_red_threshold_) {
            if (const auto distance = stop_distance(points, outputs.signal_xy, stop_offset_m_)) {
                speed_path_msgs::msg::StopPoint stop_point;
                stop_point.distance = *distance;
                stop_point_publisher_->publish(stop_point);
            }
        }
        // 下流は odom の経路を毎周期 base_link へ引き直す。観測時刻の姿勢で odom へ固定する
        local_path_publisher_->publish(std::make_unique<nav_msgs::msg::Path>(make_path(to_world(points, *pose), "odom", msg->header.stamp)));
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "経路の有効点が足りないため出力しません(slot=%s)", slot_name(slot));
    }

    // サブスクライバがいる場合のみ実行
    if (marker_publisher_->get_subscription_count() > 0) {
        marker_publisher_->publish(make_marker_array(points, msg->header));
    }
    if (intrinsics_ && debug_image_publisher_->get_subscription_count() > 0) {
        debug_image_publisher_->publish(make_debug_image(source, points, msg->header));
    }
}

visualization_msgs::msg::MarkerArray::UniquePtr VectorE2ePlannerNode::make_marker_array(
    const std::vector<Point2D>& points, const std_msgs::msg::Header& header) const {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp    = header.stamp;
    delete_marker.header.frame_id = "base_link";
    delete_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(delete_marker);

    visualization_msgs::msg::Marker line_marker;
    line_marker.header             = delete_marker.header;
    line_marker.ns                 = "vector_e2e";
    line_marker.id                 = 0;
    line_marker.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action             = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x            = 0.08;
    line_marker.color.r            = 0.0F;
    line_marker.color.g            = 0.9F;
    line_marker.color.b            = 0.2F;
    line_marker.color.a            = 1.0F;
    line_marker.points.reserve(points.size());
    for (const Point2D& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        line_marker.points.push_back(p);
    }

    marker_array->markers.push_back(std::move(line_marker));
    return marker_array;
}

sensor_msgs::msg::Image::UniquePtr VectorE2ePlannerNode::make_debug_image(
    const cv::Mat&               source,
    const std::vector<Point2D>&  points,
    const std_msgs::msg::Header& header) const {
    cv::Mat canvas = source.clone();
    std::vector<cv::Point> pixels;
    for (const Point2D& point : points) {
        cv::Point2f pixel;
        if (!camera_utility::pointToPixel(
                tf2::Vector3(point.x, point.y, 0.0), *intrinsics_, base_T_camera_, pixel)) {
            continue;
        }
        pixels.emplace_back(static_cast<int>(pixel.x), static_cast<int>(pixel.y));
    }
    if (pixels.size() >= 2U) {
        cv::polylines(canvas, pixels, false, cv::Scalar(0, 220, 0), 3, cv::LINE_AA);
    }

    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, canvas).toImageMsg(*message);
    return message;
}

}  // namespace vector_e2e_planner
