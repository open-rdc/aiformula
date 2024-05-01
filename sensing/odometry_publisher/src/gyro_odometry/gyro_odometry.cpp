#include "gyro_odometry.hpp"

namespace aiformula {

GyroOdometry::GyroOdometry() : Node("gyro_odometry"), yaw_(0.), angular_velocity_z_(0.) {
    getRosParams();
    initValues();
    printParam();
}

void GyroOdometry::getRosParams() {
    // launch
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    robot_frame_id_ = getRosParameter<std::string>(this, "robot_frame_id");

    // wheel.yaml
    tire_diameter_ = getRosParameter<double>(this, "wheel.diameter");
    tire_tread_ = getRosParameter<double>(this, "wheel.tread");
}

void GyroOdometry::initValues() {
    const int buffer_size = 10;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "sub_imu_topic_name", buffer_size, std::bind(&GyroOdometry::imuCallback, this, std::placeholders::_1));
    can_frame_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "sub_can_frame_topic_name", buffer_size,
        std::bind(&GyroOdometry::canFrameCallback, this, std::placeholders::_1));
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pub_odometry_topic_name", buffer_size);

    odometry_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void GyroOdometry::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);

    RCLCPP_INFO(this->get_logger(), "(launch)");
    RCLCPP_INFO(this->get_logger(), "  odom_frame_id_  = %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_frame_id_ = %s", robot_frame_id_.c_str());

    RCLCPP_INFO(this->get_logger(), "(wheel.yaml)");
    RCLCPP_INFO(this->get_logger(), "  tire_diameter_ = %.3lf [m]", tire_diameter_);
    RCLCPP_INFO(this->get_logger(), "  tire_tread_    = %.3lf [m]", tire_tread_);
}

void GyroOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Imu !");
    static sensor_msgs::msg::Imu::SharedPtr msg_old = msg;
    static double yaw_offset = getYaw(msg->orientation);

    yaw_ = getYaw(msg->orientation) - yaw_offset;
    angular_velocity_z_ = msg->angular_velocity.z;
}

void GyroOdometry::canFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->id != aiformula::odometry_publisher::RPM_ID) return;

    const aiformula::odometry_publisher::Wheel<unsigned int> rpm(msg->data[4], msg->data[0]);
    const aiformula::odometry_publisher::Wheel<double> vel =
        rpm * odometry_publisher::MINUTE_TO_SECOND * tire_diameter_ * M_PI;
    const double vel_ave = (vel.left + vel.right) * 0.5;

    static double stamp_old = toTimeStampDouble(msg->header);
    const double stamp_now = toTimeStampDouble(msg->header);
    const double dt = stamp_now - stamp_old;
    stamp_old = stamp_now;

    const double vel_x = vel_ave * cos(yaw_);
    const double vel_y = vel_ave * sin(yaw_);
    static double pos_x = 0.0, pos_y = 0.0;
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = robot_frame_id_;
    odometry.pose.pose.position = toPointMsg(pos_x, pos_y, 0.0);
    odometry.pose.pose.orientation = toQuaternionMsg(0.0, 0.0, yaw_);
    odometry.twist.twist.linear = toVector3Msg(vel_x, vel_y, 0.0);
    odometry.twist.twist.angular = toVector3Msg(0.0, 0.0, angular_velocity_z_);
    odometry_pub_->publish(odometry);
    aiformula::odometry_publisher::broadcastTf(odometry_br_, odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Pubscribe Odometry !");
}

}  // namespace aiformula
