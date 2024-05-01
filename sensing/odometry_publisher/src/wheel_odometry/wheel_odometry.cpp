#include "wheel_odometry.hpp"

namespace aiformula {

WheelOdometry::WheelOdometry() : Node("wheel_odometry") {
    getRosParams();
    initValues();
    printParam();
}

void WheelOdometry::getRosParams() {
    // launch
    odom_frame_id_ = getRosParameter<std::string>(this, "odom_frame_id");
    robot_frame_id_ = getRosParameter<std::string>(this, "robot_frame_id");

    // odometry_publisher.yaml
    tire_diameter_ = getRosParameter<double>(this, "wheel.diameter");
    tire_tread_ = getRosParameter<double>(this, "wheel.tread");
}

void WheelOdometry::initValues() {
    const int buffer_size = 10;
    can_frame_sub_ = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "sub_can_frame_topic_name", buffer_size,
        std::bind(&WheelOdometry::canFrameCallback, this, std::placeholders::_1));
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pub_odometry_topic_name", buffer_size);

    odometry_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void WheelOdometry::printParam() const {
    RCLCPP_INFO(this->get_logger(), "[%s] ===============", __func__);

    RCLCPP_INFO(this->get_logger(), "(launch)");
    RCLCPP_INFO(this->get_logger(), "  odom_frame_id_  = %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_frame_id_ = %s", robot_frame_id_.c_str());

    RCLCPP_INFO(this->get_logger(), "(odometry_publisher.yaml)");
    RCLCPP_INFO(this->get_logger(), "  tire_diameter_ = %.3lf [m]", tire_diameter_);
    RCLCPP_INFO(this->get_logger(), "  tire_tread_    = %.3lf [m]", tire_tread_);
}

void WheelOdometry::canFrameCallback(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Subscribe Can Frame !");
    if (msg->canid != aiformula::odometry_publisher::RPM_ID) return;

    const aiformula::odometry_publisher::Wheel<unsigned int> rpm(msg->candata[4], msg->candata[0]);
    const aiformula::odometry_publisher::Wheel<double> vel =
        rpm * aiformula::odometry_publisher::MINUTE_TO_SECOND * tire_diameter_ * M_PI;
    const double vel_ave = (vel.left + vel.right) * 0.5;
    const double w = (vel.right - vel.left) / tire_tread_;

    static double stamp_old = toTimeStampDouble(msg->header);
    const double stamp_now = toTimeStampDouble(msg->header);
    const double dt = stamp_now - stamp_old;
    stamp_old = stamp_now;

    static double yaw = 0.0;
    yaw += w * dt;
    const double vel_x = vel_ave * cos(yaw);
    const double vel_y = vel_ave * sin(yaw);
    static double pos_x = 0.0, pos_y = 0.0;
    pos_x += vel_x * dt;
    pos_y += vel_y * dt;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = robot_frame_id_;
    odometry.pose.pose.position = toPointMsg(pos_x, pos_y, 0.0);
    odometry.pose.pose.orientation = toQuaternionMsg(0.0, 0.0, yaw);
    odometry.twist.twist.linear = toVector3Msg(vel_x, vel_y, 0.0);
    odometry.twist.twist.angular = toVector3Msg(0.0, 0.0, w);
    odometry_pub_->publish(odometry);
    aiformula::odometry_publisher::broadcastTf(odometry_br_, odometry);
    RCLCPP_INFO_ONCE(this->get_logger(), "Pubscribe Odometry !");
}

}  // namespace aiformula
