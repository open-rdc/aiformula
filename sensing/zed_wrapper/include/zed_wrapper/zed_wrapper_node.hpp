#pragma once

#include "zed_wrapper/visibility_control.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace zed_wrapper
{

class ZED_WRAPPER_PUBLIC ZedWrapperNode : public rclcpp::Node
{
public:
    explicit ZedWrapperNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    explicit ZedWrapperNode(
        const std::string & name_space,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ZedWrapperNode();

private:
    void grab_callback();

    struct Implementation;
    std::unique_ptr<Implementation> implementation_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
