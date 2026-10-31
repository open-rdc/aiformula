#include "camera_utility/camera_utility.hpp"

#include <cmath>

#include <utilities/utils.hpp>

namespace camera_utility
{

CameraIntrinsics fromCameraInfo(const sensor_msgs::msg::CameraInfo& msg)
{
    return CameraIntrinsics{
        static_cast<int>(msg.width), static_cast<int>(msg.height),
        msg.k[0], msg.k[4], msg.k[2], msg.k[5]};
}

tf2::Transform getBaseTCamera(rclcpp::Node& node)
{
    const tf2::Vector3 position(
        node.get_parameter("camera.position.x").as_double(),
        node.get_parameter("camera.position.y").as_double(),
        node.get_parameter("camera.position.z").as_double());
    return makeTf2Transform(
        position,
        node.get_parameter("camera.orientation.roll").as_double(),
        node.get_parameter("camera.orientation.pitch").as_double(),
        node.get_parameter("camera.orientation.yaw").as_double());
}

tf2::Transform makeTf2Transform(
    const tf2::Vector3& position,
    const double roll_deg,
    const double pitch_deg,
    const double yaw_deg)
{
    tf2::Quaternion rotation;
    rotation.setRPY(utils::dtor(roll_deg), utils::dtor(pitch_deg), utils::dtor(yaw_deg));
    return tf2::Transform(rotation, position);
}

bool pixelToPoint(
    const cv::Point2f& pixel,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    tf2::Vector3& base_point,
    const double ground_z)
{
    const tf2::Vector3 camera_vec(
        (static_cast<double>(pixel.x) - intrinsics.cx) / intrinsics.fx,
        (static_cast<double>(pixel.y) - intrinsics.cy) / intrinsics.fy,
        1.0);
    const tf2::Vector3 base_vec = tf2::quatRotate(base_T_camera.getRotation(), camera_vec);
    const double scale = (ground_z - base_T_camera.getOrigin().z()) / base_vec.z();
    if (!std::isfinite(scale) || scale <= 0.0) {
        return false;
    }
    base_point = base_T_camera * (scale * camera_vec);
    return true;
}

std::vector<tf2::Vector3> pixelsToPoints(
    const std::vector<cv::Point2f>& pixels,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    const double ground_z)
{
    std::vector<tf2::Vector3> base_points;
    base_points.reserve(pixels.size());
    for (const auto& pixel : pixels) {
        tf2::Vector3 base_point;
        if (pixelToPoint(pixel, intrinsics, base_T_camera, base_point, ground_z)) {
            base_points.push_back(base_point);
        }
    }
    return base_points;
}

bool pointToPixel(
    const tf2::Vector3& base_point,
    const CameraIntrinsics& intrinsics,
    const tf2::Transform& base_T_camera,
    cv::Point2f& pixel)
{
    const tf2::Transform camera_T_base = base_T_camera.inverse();
    const tf2::Vector3 camera_point = camera_T_base * base_point;
    if (camera_point.z() <= 0.0) {
        return false;
    }
    pixel.x = static_cast<float>(intrinsics.fx * camera_point.x() / camera_point.z() + intrinsics.cx);
    pixel.y = static_cast<float>(intrinsics.fy * camera_point.y() / camera_point.z() + intrinsics.cy);
    return true;
}

}
