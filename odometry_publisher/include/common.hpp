#ifndef COMMON_HPP
#define COMMON_HPP

// ROS
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/impl/utils.h>
#include <rclcpp/rclcpp.hpp>

// ROS msg
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

namespace aiformula {
namespace odometry_publisher {

template <typename T>
struct Wheel {
    Wheel(const T& left, const T& right) : left(left), right(right) {}
    Wheel<double> operator*(const double& rhs) const { return {left * rhs, right * rhs}; }
    T left, right;
};
const double MINUTE_TO_SECOND = 0.016667;  // = 1/60
const double DEGREE_TO_RADIAN = M_PI / 180.0;
const double RADIAN_TO_DEGREE = 180.0 / M_PI;

const int RPM_ID = 1809;

void broadcastTf(std::unique_ptr<tf2_ros::TransformBroadcaster>& odometry_br, const nav_msgs::msg::Odometry& odom);

}  // namespace odometry_publisher

geometry_msgs::msg::Point toPointMsg(const double& x, const double& y, const double& z);
geometry_msgs::msg::Quaternion toQuaternionMsg(const double& roll, const double& pitch, const double& yaw);
geometry_msgs::msg::Vector3 toVector3Msg(const double& x, const double& y, const double& z);
geometry_msgs::msg::Vector3 toVector3Msg(const geometry_msgs::msg::Point& point);

inline double toTimeStampDouble(const std_msgs::msg::Header& header) {
    return header.stamp.sec + static_cast<double>(header.stamp.nanosec) / 1e9;
}

inline double getYaw(const geometry_msgs::msg::Quaternion& quat_msg) {
    return tf2::impl::getYaw(tf2::impl::toQuaternion(quat_msg));
}

// Declaration ==========================================================================================
template <typename T>
T getParameterAsType(rclcpp::Node* node_ptr, const std::string& param_name);

// Implementation =======================================================================================
template <>
inline bool getParameterAsType<bool>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool();
}

template <>
inline int getParameterAsType<int>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_int();
}

template <>
inline double getParameterAsType<double>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double();
}

template <>
inline std::string getParameterAsType<std::string>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string();
}

template <>
inline std::vector<bool> getParameterAsType<std::vector<bool>>(rclcpp::Node* node_ptr, const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_bool_array();
}

template <>
inline std::vector<long int> getParameterAsType<std::vector<long int>>(rclcpp::Node* node_ptr,
                                                                       const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_integer_array();
}

template <>
inline std::vector<double> getParameterAsType<std::vector<double>>(rclcpp::Node* node_ptr,
                                                                   const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_double_array();
}

template <>
inline std::vector<std::string> getParameterAsType<std::vector<std::string>>(rclcpp::Node* node_ptr,
                                                                             const std::string& param_name) {
    return node_ptr->get_parameter(param_name).as_string_array();
}

/**
 * @brief If the parameter named `param_name` has no value set, output error statement and shutdown ros.
 * If `param_name` type does not match `T`, output error statement and shutdown ros.
 *
 * @param[in] node_ptr node pointer
 * @param[in] param_name Rosparam Name
 * @return Value set for the parameter named `param_name`.
 *
 * @note Usage: `double ret = getRosParameter<double>(this, "test.value");`
 */
template <typename T>
T getRosParameter(rclcpp::Node* node_ptr, const std::string& param_name) {
    if (!node_ptr->has_parameter(param_name)) {
        node_ptr->declare_parameter(param_name);
    }
    try {
        return getParameterAsType<T>(node_ptr, param_name);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' is not declared", e.what());
        rclcpp::shutdown();
        exit(1);
    } catch (const rclcpp::ParameterTypeException& e) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Parameter '%s' has an incorrect type: %s", param_name.c_str(), e.what());
        rclcpp::shutdown();
        exit(1);
    }
}

}  // namespace aiformula

#endif  // COMMON_HPP
