#include "zed_wrapper/zed_wrapper_node.hpp"

#include <cstring>
#include <stdexcept>

#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sl/Camera.hpp>

namespace zed_wrapper
{

struct ZedWrapperNode::Implementation
{
    sl::Camera           zed;
    sl::RuntimeParameters runtime_params;
    sl::Resolution       publish_resolution;
};

namespace
{

struct CaptureConfig
{
    sl::RESOLUTION capture_resolution;
    sl::Resolution publish_resolution;
};

CaptureConfig capture_config_from(const std::string & resolution)
{
    if (resolution == "HD1200") { return {sl::RESOLUTION::HD1200, sl::Resolution(1920, 1200)}; }
    if (resolution == "HD1080") { return {sl::RESOLUTION::HD1080, sl::Resolution(1920, 1080)}; }
    if (resolution == "SVGA")   { return {sl::RESOLUTION::SVGA,   sl::Resolution(960, 600)}; }
    if (resolution == "nHD")    { return {sl::RESOLUTION::HD1080, sl::Resolution(640, 360)}; }

    throw std::invalid_argument("ZedWrapperNode: unsupported camera.size: " + resolution);
}

sensor_msgs::msg::CameraInfo camera_info_from(sl::Camera & zed, const sl::Resolution & resolution)
{
    const auto calibration = zed.getCameraInformation(resolution).camera_configuration.calibration_parameters.left_cam;

    sensor_msgs::msg::CameraInfo msg;
    msg.header.frame_id = "camera_depth_link";
    msg.width  = static_cast<uint32_t>(resolution.width);
    msg.height = static_cast<uint32_t>(resolution.height);
    msg.distortion_model = "plumb_bob";
    msg.d.assign(5, 0.0);
    msg.k = {calibration.fx, 0.0, calibration.cx,
             0.0, calibration.fy, calibration.cy,
             0.0, 0.0, 1.0};
    msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.p = {calibration.fx, 0.0, calibration.cx, 0.0,
             0.0, calibration.fy, calibration.cy, 0.0,
             0.0, 0.0, 1.0, 0.0};
    return msg;
}

sl::DEPTH_MODE parse_depth_mode(const std::string & s)
{
    if (s == "NONE")        { return sl::DEPTH_MODE::NONE; }
    if (s == "PERFORMANCE") { return sl::DEPTH_MODE::PERFORMANCE; }
    if (s == "QUALITY")     { return sl::DEPTH_MODE::QUALITY; }
    if (s == "ULTRA")       { return sl::DEPTH_MODE::ULTRA; }
    if (s == "NEURAL")      { return sl::DEPTH_MODE::NEURAL; }
    if (s == "NEURAL_PLUS") { return sl::DEPTH_MODE::NEURAL_PLUS; }

    throw std::invalid_argument("ZedWrapperNode: unsupported depth_mode: " + s);
}

}

ZedWrapperNode::ZedWrapperNode(const rclcpp::NodeOptions & options)
: ZedWrapperNode("", options) {}

ZedWrapperNode::ZedWrapperNode(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("zed_wrapper_node", name_space, options),
  implementation_(std::make_unique<Implementation>())
{
    const int    fps   = get_parameter("fps").as_int();
    const auto   resolution = get_parameter("camera.size").as_string();
    const auto   depth_mode = get_parameter("depth.mode").as_string();
    const int    confidence = get_parameter("depth.confidence_threshold").as_int();

    const auto capture_config = capture_config_from(resolution);
    implementation_->publish_resolution = capture_config.publish_resolution;

    sl::InitParameters init_params;
    init_params.camera_resolution = capture_config.capture_resolution;
    init_params.camera_fps        = fps;
    init_params.depth_mode        = parse_depth_mode(depth_mode);
    init_params.coordinate_units  = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    const sl::ERROR_CODE error_code = implementation_->zed.open(init_params);
    if (error_code != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error(std::string("ZedWrapperNode: camera open failed: ") + sl::toString(error_code).c_str());
    }

    implementation_->runtime_params.confidence_threshold = confidence;

    // odom配信に必要なポジショナルトラッキングを有効化
    const sl::ERROR_CODE tracking_error = implementation_->zed.enablePositionalTracking(sl::PositionalTrackingParameters());
    if (tracking_error != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN(get_logger(), "ZedWrapperNode: enable positional tracking failed: %s", sl::toString(tracking_error).c_str());
    }

    camera_info_msg_ = camera_info_from(implementation_->zed, capture_config.publish_resolution);

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", rclcpp::QoS(10));
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/zed/zed_node/point_cloud", rclcpp::QoS(10));
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>("/zed/zed_node/rgb/camera_info", rclcpp::QoS(10));
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/zed/zed_node/odom", rclcpp::QoS(10));

    const auto period = std::chrono::milliseconds(1000 / fps);
    timer_ = create_wall_timer(period, [this]() { grab_callback(); });

    RCLCPP_INFO(get_logger(),"ZedWrapperNode initialized (fps=%d, resolution=%s, depth=%s)", fps, resolution.c_str(), depth_mode.c_str());
}

ZedWrapperNode::~ZedWrapperNode()
{
    implementation_->zed.close();
}

void ZedWrapperNode::grab_callback()
{
    const sl::ERROR_CODE error_code = implementation_->zed.grab(implementation_->runtime_params);
    if (error_code != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "ZedWrapperNode: grab failed: %s", sl::toString(error_code).c_str());
        return;
    }

    const rclcpp::Time stamp = now();

    camera_info_msg_.header.stamp = stamp;
    camera_info_publisher_->publish(camera_info_msg_);

    // subscriberがいない場合は処理しない
    if (image_publisher_->get_subscription_count() > 0) {
        sl::Mat left_image;
        implementation_->zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU, implementation_->publish_resolution);

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp    = stamp;
        msg->header.frame_id = "camera_depth_link";
        msg->width    = static_cast<uint32_t>(left_image.getWidth());
        msg->height   = static_cast<uint32_t>(left_image.getHeight());
        msg->encoding = "bgra8";
        msg->step     = static_cast<uint32_t>(left_image.getStepBytes());
        const size_t nbytes = msg->height * msg->step;
        msg->data.resize(nbytes);
        std::memcpy(msg->data.data(), left_image.getPtr<sl::uchar1>(), nbytes);
        image_publisher_->publish(std::move(msg));
    }

    // subscriberがいない場合は処理しない
    if (pointcloud_publisher_->get_subscription_count() > 0) {
        sl::Mat pc_mat;
        implementation_->zed.retrieveMeasure(pc_mat, sl::MEASURE::XYZRGBA, sl::MEM::CPU, implementation_->publish_resolution);

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp    = stamp;
        msg->header.frame_id = "camera_depth_link";
        msg->height     = static_cast<uint32_t>(pc_mat.getHeight());
        msg->width      = static_cast<uint32_t>(pc_mat.getWidth());
        msg->is_dense   = false;
        msg->point_step = 16;
        msg->row_step   = msg->point_step * msg->width;

        msg->fields.resize(4);
        msg->fields[0].name = "x";    msg->fields[0].offset = 0;
        msg->fields[1].name = "y";    msg->fields[1].offset = 4;
        msg->fields[2].name = "z";    msg->fields[2].offset = 8;
        msg->fields[3].name = "rgba"; msg->fields[3].offset = 12;
        for (auto & f : msg->fields) {
            f.datatype = sensor_msgs::msg::PointField::FLOAT32;
            f.count    = 1;
        }

        const size_t nbytes = static_cast<size_t>(msg->height) * msg->row_step;
        msg->data.resize(nbytes);
        std::memcpy(msg->data.data(), pc_mat.getPtr<sl::uchar1>(), nbytes);
        pointcloud_publisher_->publish(std::move(msg));
    }

    // subscriberがいない場合は処理しない
    if (odometry_publisher_->get_subscription_count() > 0) {
        sl::Pose zed_pose;
        const sl::POSITIONAL_TRACKING_STATE tracking_state =
            implementation_->zed.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);

        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.stamp    = stamp;
        msg->header.frame_id = "odom";
        msg->child_frame_id  = "base_link";

        // ZEDのトラッキング状態がOKでない場合は、位置情報を更新しない
        if (sl::POSITIONAL_TRACKING_STATE::OK != tracking_state) {
            RCLCPP_WARN(get_logger(), "ZedWrapperNode: tracking state is not OK: %s", sl::toString(tracking_state).c_str());
        }else {
            msg->pose.pose.position.x = zed_pose.getTranslation().tx;
            msg->pose.pose.position.y = zed_pose.getTranslation().ty;
            msg->pose.pose.position.z = zed_pose.getTranslation().tz;

            msg->pose.pose.orientation.x = zed_pose.getOrientation().ox;
            msg->pose.pose.orientation.y = zed_pose.getOrientation().oy;
            msg->pose.pose.orientation.z = zed_pose.getOrientation().oz;
            msg->pose.pose.orientation.w = zed_pose.getOrientation().ow;
        }

        odometry_publisher_->publish(std::move(msg));
    }

}

}
