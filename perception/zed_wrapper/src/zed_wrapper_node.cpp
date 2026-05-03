#include "zed_wrapper/zed_wrapper_node.hpp"

#include <cstring>
#include <stdexcept>

#include <sensor_msgs/msg/point_field.hpp>
#include <sl/Camera.hpp>

namespace zed_wrapper
{

// --- PIMPL body -----------------------------------------------------------
struct ZedWrapperNode::Impl
{
    sl::Camera           zed;
    sl::RuntimeParameters runtime_params;
};

// --- helpers --------------------------------------------------------------
namespace
{

sl::RESOLUTION parse_resolution(const std::string & s)
{
    if (s == "HD1080") { return sl::RESOLUTION::HD1080; }
    if (s == "VGA")    { return sl::RESOLUTION::VGA; }
    return sl::RESOLUTION::HD720;
}

sl::DEPTH_MODE parse_depth_mode(const std::string & s)
{
    if (s == "QUALITY") { return sl::DEPTH_MODE::QUALITY; }
    if (s == "ULTRA")   { return sl::DEPTH_MODE::ULTRA; }
    return sl::DEPTH_MODE::PERFORMANCE;
}

}  // namespace

// --- constructors / destructor --------------------------------------------
ZedWrapperNode::ZedWrapperNode(const rclcpp::NodeOptions & options)
: ZedWrapperNode("", options) {}

ZedWrapperNode::ZedWrapperNode(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("zed_wrapper_node", name_space, options),
  impl_(std::make_unique<Impl>())
{
    const int    grab_fps   = get_parameter("grab_fps").as_int();
    const auto   resolution = get_parameter("resolution").as_string();
    const auto   depth_mode = get_parameter("depth_mode").as_string();
    const int    confidence = get_parameter("confidence_threshold").as_int();
    const int    serial_num = get_parameter("serial_number").as_int();
    camera_frame_id_        = get_parameter("camera_frame_id").as_string();

    if (grab_fps <= 0) {
        throw std::invalid_argument("ZedWrapperNode: grab_fps must be > 0");
    }

    sl::InitParameters init_params;
    init_params.camera_resolution = parse_resolution(resolution);
    init_params.camera_fps        = grab_fps;
    init_params.depth_mode        = parse_depth_mode(depth_mode);
    init_params.coordinate_units  = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    if (serial_num != 0) {
        init_params.input.setFromSerialNumber(serial_num);
    }

    const sl::ERROR_CODE ec = impl_->zed.open(init_params);
    if (ec != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error(
            std::string("ZedWrapperNode: camera open failed: ") + sl::toString(ec).c_str());
    }

    impl_->runtime_params.confidence_threshold = confidence;
    camera_info_cache_ = build_camera_info();

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/image_rect_color", rclcpp::QoS(10));
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud", rclcpp::QoS(10));
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/zed/zed_node/rgb/camera_info", rclcpp::QoS(10));

    const auto period = std::chrono::milliseconds(1000 / grab_fps);
    timer_ = create_wall_timer(period, [this]() { grab_callback(); });

    RCLCPP_INFO(get_logger(), "ZedWrapperNode initialized (fps=%d, res=%s, depth=%s)",
        grab_fps, resolution.c_str(), depth_mode.c_str());
}

ZedWrapperNode::~ZedWrapperNode()
{
    impl_->zed.close();
}

// --- timer callback -------------------------------------------------------
void ZedWrapperNode::grab_callback()
{
    const sl::ERROR_CODE ec = impl_->zed.grab(impl_->runtime_params);
    if (ec != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "ZedWrapperNode: grab failed: %s", sl::toString(ec).c_str());
        return;
    }

    const rclcpp::Time stamp = now();

    // Left RGB image (BGRA)
    sl::Mat left_image;
    impl_->zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
    {
        sensor_msgs::msg::Image msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = camera_frame_id_;
        msg.width    = static_cast<uint32_t>(left_image.getWidth());
        msg.height   = static_cast<uint32_t>(left_image.getHeight());
        msg.encoding = "bgra8";
        msg.step     = static_cast<uint32_t>(left_image.getStepBytes());
        const size_t nbytes = msg.height * msg.step;
        msg.data.resize(nbytes);
        std::memcpy(msg.data.data(), left_image.getPtr<sl::uchar1>(), nbytes);
        image_publisher_->publish(msg);
    }

    // XYZRGBA point cloud
    sl::Mat pc_mat;
    impl_->zed.retrieveMeasure(pc_mat, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = camera_frame_id_;
        msg.height     = static_cast<uint32_t>(pc_mat.getHeight());
        msg.width      = static_cast<uint32_t>(pc_mat.getWidth());
        msg.is_dense   = false;
        msg.point_step = 16;  // 4 x float32: x, y, z, rgba
        msg.row_step   = msg.point_step * msg.width;

        msg.fields.resize(4);
        msg.fields[0].name = "x";    msg.fields[0].offset = 0;
        msg.fields[1].name = "y";    msg.fields[1].offset = 4;
        msg.fields[2].name = "z";    msg.fields[2].offset = 8;
        msg.fields[3].name = "rgba"; msg.fields[3].offset = 12;
        for (auto & f : msg.fields) {
            f.datatype = sensor_msgs::msg::PointField::FLOAT32;
            f.count    = 1;
        }

        const size_t nbytes = static_cast<size_t>(msg.height) * msg.row_step;
        msg.data.resize(nbytes);
        std::memcpy(msg.data.data(), pc_mat.getPtr<sl::uchar1>(), nbytes);
        pointcloud_publisher_->publish(msg);
    }

    // Camera info (constant intrinsics, only stamp changes each frame)
    camera_info_cache_.header.stamp = stamp;
    camera_info_publisher_->publish(camera_info_cache_);
}

// --- camera info construction (called once after open) --------------------
sensor_msgs::msg::CameraInfo ZedWrapperNode::build_camera_info()
{
    const sl::CameraInformation cam_info = impl_->zed.getCameraInformation();
    const auto & calib = cam_info.camera_configuration.calibration_parameters.left_cam;
    const auto & res   = cam_info.camera_configuration.resolution;

    sensor_msgs::msg::CameraInfo msg;
    msg.header.frame_id  = camera_frame_id_;
    msg.width            = static_cast<uint32_t>(res.width);
    msg.height           = static_cast<uint32_t>(res.height);
    msg.distortion_model = "plumb_bob";

    // D: k1, k2, p1, p2, k3
    msg.d = {calib.disto[0], calib.disto[1], calib.disto[2], calib.disto[3], calib.disto[4]};

    // K: row-major 3x3
    msg.k = {
        calib.fx, 0.0,      calib.cx,
        0.0,      calib.fy, calib.cy,
        0.0,      0.0,      1.0
    };

    // R: identity (images are already rectified by ZED SDK)
    msg.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};

    // P: row-major 3x4
    msg.p = {
        calib.fx, 0.0,      calib.cx, 0.0,
        0.0,      calib.fy, calib.cy, 0.0,
        0.0,      0.0,      1.0,      0.0
    };

    return msg;
}

}  // namespace zed_wrapper
