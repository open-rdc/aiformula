#include "zed_wrapper/zed_wrapper_node.hpp"

#include <cstring>
#include <stdexcept>

#include <camera_utility/camera_intrinsics.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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

sl::RESOLUTION capture_resolution_from(const std::string & size)
{
    if (size == "nHD") { return sl::RESOLUTION::HD1080; }
    return sl::RESOLUTION::SVGA;
}

sl::DEPTH_MODE parse_depth_mode(const std::string & s)
{
    if (s == "NONE")        { return sl::DEPTH_MODE::NONE; }
    if (s == "PERFORMANCE") { return sl::DEPTH_MODE::PERFORMANCE; }
    if (s == "QUALITY")     { return sl::DEPTH_MODE::QUALITY; }
    if (s == "ULTRA")       { return sl::DEPTH_MODE::ULTRA; }
    if (s == "NEURAL")      { return sl::DEPTH_MODE::NEURAL; }
    if (s == "NEURAL_PLUS") { return sl::DEPTH_MODE::NEURAL_PLUS; }
    throw std::invalid_argument(
        "ZedWrapperNode: unsupported depth_mode: " + s);
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
    const auto   size = get_parameter("camera.size").as_string();
    const auto   depth_mode = get_parameter("depth.mode").as_string();
    const int    confidence = get_parameter("depth.confidence_threshold").as_int();

    const auto intrinsics = camera_utility::getCameraIntrinsics(size);
    implementation_->publish_resolution = sl::Resolution(
        static_cast<size_t>(intrinsics.width), static_cast<size_t>(intrinsics.height));

    sl::InitParameters init_params;
    init_params.camera_resolution = capture_resolution_from(size);
    init_params.camera_fps        = fps;
    init_params.depth_mode        = parse_depth_mode(depth_mode);
    init_params.coordinate_units  = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    const sl::ERROR_CODE ec = implementation_->zed.open(init_params);
    if (ec != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error(
            std::string("ZedWrapperNode: camera open failed: ") + sl::toString(ec).c_str());
    }

    implementation_->runtime_params.confidence_threshold = confidence;

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/image_rect_color", rclcpp::QoS(10));
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud", rclcpp::QoS(10));

    const auto period = std::chrono::milliseconds(1000 / fps);
    timer_ = create_wall_timer(period, [this]() { grab_callback(); });

    RCLCPP_INFO(get_logger(), "ZedWrapperNode initialized (fps=%d, size=%s, depth=%s)",
        fps, size.c_str(), depth_mode.c_str());
}

ZedWrapperNode::~ZedWrapperNode()
{
    implementation_->zed.close();
}

void ZedWrapperNode::grab_callback()
{
    const sl::ERROR_CODE ec = implementation_->zed.grab(implementation_->runtime_params);
    if (ec != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "ZedWrapperNode: grab failed: %s", sl::toString(ec).c_str());
        return;
    }

    const rclcpp::Time stamp = now();

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
}

}
