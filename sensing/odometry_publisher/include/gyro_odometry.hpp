#ifndef GYRO_ODOMETRY_HPP
#define GYRO_ODOMETRY_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Original
#include "common.hpp"
#include "get_ros_parameter.hpp"  // common_cpp librtary
#include "to_geometry_msgs.hpp"   // common_cpp librtary
#include "util.hpp"               // common_cpp librtary

namespace aiformula {

class GyroOdometry : public rclcpp::Node {
public:
    GyroOdometry();
    ~GyroOdometry() = default;

private:
    void getRosParams();
    void initValues();
    void printParam() const;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg);

    std::string odom_frame_id_;
    std::string robot_frame_id_;
    double tire_diameter_;
    double tire_tread_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odometry_br_;

    double yaw_;
    double angular_velocity_z_;
};

}  // namespace aiformula

#endif  // GYRO_ODOMETRY_HPP
