#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <topomap_msgs/msg/topo_map.hpp>

#include "topomap_localizer/visibility_control.h"

namespace topomap_localizer
{

class TopomapLocalizerNode : public rclcpp::Node
{
public:
    TOPOMAP_LOCALIZER_PUBLIC
    explicit TopomapLocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct Target
    {
        uint64_t id;
        geometry_msgs::msg::Point position;
        std::string command;
        double hout;
        size_t index;
    };

    void topomap_callback(const topomap_msgs::msg::TopoMap::ConstSharedPtr msg);
    void timer_callback();

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    topomap_msgs::msg::TopoMap::ConstSharedPtr topomap_;
    std::vector<Target> targets_;
    std::vector<geometry_msgs::msg::Point> route_points_;
    sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_;

    const bool init_gnss_flag_;
    bool autonomous_ = false;
    bool initialized_ = false;
    double theta_ = 0.0;
    double tx_ = 0.0;
    double ty_ = 0.0;
    size_t k_ = 0;
    bool active_ = false;
    double yaw_in_ = 0.0;
    std::vector<std::pair<std::array<double, 2>, std::array<double, 2>>> anchors_;
    std::string command_;

    rclcpp::Subscription<topomap_msgs::msg::TopoMap>::SharedPtr topomap_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
