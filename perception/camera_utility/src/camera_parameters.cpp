#include "camera_utility/camera_parameters.hpp"

#include "camera_utility/ground_conversion.hpp"

namespace camera_utility
{

CameraIntrinsics getCameraIntrinsics(rclcpp::Node& node)
{
    return getCameraIntrinsics(node.get_parameter("camera.size").as_string());
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

}
