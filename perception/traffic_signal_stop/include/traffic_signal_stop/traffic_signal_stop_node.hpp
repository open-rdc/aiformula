#pragma once

#include <string>

#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <speed_path_msgs/msg/stop_point.hpp>

#include "traffic_signal_stop/visibility_control.h"

namespace traffic_signal_stop
{

class TrafficSignalStopNode : public rclcpp::Node
{
public:
    TRAFFIC_SIGNAL_STOP_PUBLIC
    explicit TrafficSignalStopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    TRAFFIC_SIGNAL_STOP_PUBLIC
    explicit TrafficSignalStopNode(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void objects_callback(const object_detection_msgs::msg::ObjectInfoArray::ConstSharedPtr msg);

    const double stop_line_offset_m_;

    rclcpp::Subscription<object_detection_msgs::msg::ObjectInfoArray>::SharedPtr objects_subscription_;
    rclcpp::Publisher<speed_path_msgs::msg::StopPoint>::SharedPtr stop_point_publisher_;
};

}
