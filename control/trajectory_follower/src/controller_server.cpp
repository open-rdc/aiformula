#include "trajectory_follower/controller_server.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_follower
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller_server_node", options),
  plugin_loader_("trajectory_follower", "trajectory_follower::ControllerPlugin")
{
    plugin_ = plugin_loader_.createSharedInstance(get_parameter("controller_plugin").as_string());
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    velocity_ = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_subscription_ = create_subscription<speed_path_msgs::msg::SpeedPath>("/planner/speed_path", rclcpp::QoS(1).best_effort(), std::bind(&ControllerServer::path_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vectornav/velocity_body", rclcpp::QoS(1).best_effort(), std::bind(&ControllerServer::velocity_callback, this, std::placeholders::_1));
    autonomous_subscription_ = create_subscription<std_msgs::msg::Bool>("/autonomous", rclcpp::QoS(10), std::bind(&ControllerServer::autonomous_callback, this, std::placeholders::_1));
    caster_data_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>("/caster_data", rclcpp::SensorDataQoS().keep_last(1), std::bind(&ControllerServer::caster_data_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", rclcpp::SensorDataQoS().keep_last(1), std::bind(&ControllerServer::imu_callback, this, std::placeholders::_1));
    command_publisher_ = create_publisher<steered_drive_msg::msg::SteeredDrive>("/cmd_vel", rclcpp::QoS(10));

    timer_ = create_wall_timer(std::chrono::milliseconds(get_parameter("interval_ms").as_int()), std::bind(&ControllerServer::timer_callback, this));
}

void ControllerServer::autonomous_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    if (msg->data && !autonomous_flag_) {
        plugin_->reset();
    }
    autonomous_flag_ = msg->data;
}

void ControllerServer::path_callback(const speed_path_msgs::msg::SpeedPath::ConstSharedPtr msg)
{
    path_ = msg;
}

void ControllerServer::velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
    velocity_ = msg;
}

void ControllerServer::caster_data_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    caster_data_ = msg;
}

void ControllerServer::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    imu_ = msg;
}

void ControllerServer::timer_callback()
{
    if (!autonomous_flag_ || !path_ || path_->points.size() < 2) {
        return;
    }

    // TF が引けないときは停止指令を出す
    if (!tf_buffer_->canTransform("base_link", path_->header.frame_id, tf2::TimePointZero)) {
        command_publisher_->publish(steered_drive_msg::msg::SteeredDrive());
        return;
    }
    const auto base_T_path = tf_buffer_->lookupTransform("base_link", path_->header.frame_id, tf2::TimePointZero);

    speed_path_msgs::msg::SpeedPath path_in_base;
    path_in_base.header.frame_id = "base_link";
    path_in_base.header.stamp = path_->header.stamp;
    path_in_base.points.reserve(path_->points.size());
    for (const auto & point : path_->points) {
        speed_path_msgs::msg::SpeedPathPoint point_base = point;
        tf2::doTransform(point.pose, point_base.pose, base_T_path);
        path_in_base.points.push_back(point_base);
    }

    if (caster_data_) {
        plugin_->setMeasuredSteer(caster_data_->data[1]);
    }

    if (imu_) {
        plugin_->setYawRate(imu_->angular_velocity.z);
        imu_.reset();
    }

    command_publisher_->publish(plugin_->computeCommand(path_in_base, velocity_->twist.twist.linear.x));
}

}
