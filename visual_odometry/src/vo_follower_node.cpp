#include "visual_odometry/vo_follower_node.hpp"

using namespace utils;

namespace vonav
{

PurePursuitFollower::PurePursuitFollower(const rclcpp::NodeOptions& options)
: rclcpp::Node("vo_pure_pursuit_follower", options)
{
    this->get_parameter_or("interval_ms", freq_ms_, 50);
    this->get_parameter_or("lookahead_gain", ld_gain_, 1.5);
    this->get_parameter_or("min_lookahead_distance", ld_min_, 0.5);
    this->get_parameter_or("max_linear_vel", v_max_, 0.5);
    this->get_parameter_or("max_angular_vel", w_max_, 1.0);

    pid_ = std::make_unique<controller::PositionPid>(freq_ms_);
    double p_gain = 1.0, i_gain = 0.0, d_gain = 0.0;
    this->get_parameter_or("p_gain", p_gain, 1.0);
    this->get_parameter_or("i_gain", i_gain, 0.0);
    this->get_parameter_or("d_gain", d_gain, 0.0);
    pid_->gain(p_gain, i_gain, d_gain);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/zed/zed_node/odom", 10,
        std::bind(&PurePursuitFollower::odomCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/vo_path", 10,
        std::bind(&PurePursuitFollower::pathCallback, this, std::placeholders::_1));

    autonomous_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/autonomous", 10,
        std::bind(&PurePursuitFollower::autonomousCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead", 10);
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(freq_ms_),
        std::bind(&PurePursuitFollower::controlLoop, this));

    RCLCPP_INFO(get_logger(), "VO Pure Pursuit Follower started");
}

/* ===================== CALLBACK ===================== */

void PurePursuitFollower::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = yawFromQuat(msg->pose.pose.orientation);

    if (!base_initialized_) {
        base_x_ = current_x_;
        base_y_ = current_y_;
        base_yaw_ = current_yaw_;
        base_initialized_ = true;
    }

    publishCurrentPose();
}

void PurePursuitFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_ = msg->poses;
    target_idx_ = 0;
    prev_idx_ = 0;
}

void PurePursuitFollower::autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    autonomous_ = msg->data;
}

/* ===================== CONTROL ===================== */

void PurePursuitFollower::controlLoop()
{
    if (!autonomous_ || path_.empty() || !base_initialized_) return;

    findLookaheadPoint();
    publishLookahead();

    double heading_error = calcHeadingError();
    double w = pid_->cycle(heading_error);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v_max_;
    cmd.angular.z = constrain(w, -w_max_, w_max_);

    if (target_idx_ >= path_.size()) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
}

/* ===================== PURE PURSUIT ===================== */

void PurePursuitFollower::findLookaheadPoint()
{
    double lookahead = ld_gain_ * v_max_ + ld_min_;

    size_t closest_idx = prev_idx_;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = prev_idx_; i < path_.size(); ++i) {
        const double dx = path_[i].pose.position.x - current_x_;
        const double dy = path_[i].pose.position.y - current_y_;
        const double dist = std::hypot(dx, dy);

        if (dist < closest_dist) {
            closest_dist = dist;
            closest_idx = i;
        }
    }

    prev_idx_ = closest_idx;

    for (size_t i = closest_idx; i < path_.size(); ++i) {
        const double dx = path_[i].pose.position.x - current_x_;
        const double dy = path_[i].pose.position.y - current_y_;
        const double dist = std::hypot(dx, dy);

        if (dist > lookahead) {
            target_idx_ = i;
            prev_idx_ = i;
            return;
        }
    }
    target_idx_ = path_.size();
}

double PurePursuitFollower::calcHeadingError()
{
    if (target_idx_ >= path_.size()) return 0.0;

    double dx = path_[target_idx_].pose.position.x - current_x_;
    double dy = path_[target_idx_].pose.position.y - current_y_;

    double target_yaw = std::atan2(dy, dx);
    double err = target_yaw - current_yaw_;

    return std::atan2(std::sin(err), std::cos(err));
}

/* ===================== VISUALIZATION ===================== */

void PurePursuitFollower::publishLookahead()
{
    if (target_idx_ >= path_.size()) return;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = path_[target_idx_].pose.position.x - base_x_;
    pose.pose.position.y = path_[target_idx_].pose.position.y - base_y_;
    lookahead_pub_->publish(pose);
}

void PurePursuitFollower::publishCurrentPose()
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";

    pose.pose.position.x = current_x_ - base_x_;
    pose.pose.position.y = current_y_ - base_y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_ - base_yaw_);
    pose.pose.orientation = tf2::toMsg(q);

    current_pose_pub_->publish(pose);
}

/* ===================== UTILS ===================== */

double PurePursuitFollower::yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tfq);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

} // namespace vonav

// ===== main =====
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<vonav::PurePursuitFollower>(
            rclcpp::NodeOptions()
        )
    );
    rclcpp::shutdown();
    return 0;
}
