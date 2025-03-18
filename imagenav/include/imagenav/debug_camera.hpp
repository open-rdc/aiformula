#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

namespace debug_camera{
class DebugCamera : public rclcpp::Node{
public:
    explicit DebugCamera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit DebugCamera(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~DebugCamera();
private:

    ObjectDetectionParameters detection_parameters;
};
}
