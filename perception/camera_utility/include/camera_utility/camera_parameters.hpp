#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>

#include "camera_utility/camera_intrinsics.hpp"

namespace camera_utility
{

CameraIntrinsics getCameraIntrinsics(rclcpp::Node& node);

tf2::Transform getBaseTCamera(rclcpp::Node& node);

}
