#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "utilities/position_pid.hpp"
#include "utilities/utils.hpp"

#include <cmath>

using namespace utils;

namespace vonav
{

class PurePursuitFollower : public rclcpp::Node
{
public:
    explicit PurePursuitFollower(const rclcpp::NodeOptions& options)
    : Node("vo_pure_pursuit_follower", options),
      freq_ms_(declare_parameter("interval_ms", 50)),
      ld_gain_(declare_parameter("lookahead_gain", 1.5)),
      ld_min_(declare_parameter("min_lookahead_distance", 0.5)),
      v_max_(declare_parameter("max_linear_vel", 0.5)),
      w_max_(declare_parameter("max_angular_vel", 1.0)),
      pid_(freq_ms_)
    {
        pid_.gain(
            declare_parameter("p_gain", 1.0),
            declare_parameter("i_gain", 0.0),
            declare_parameter("d_gain", 0.0)
        );

        /* ---------- Subscriber ---------- */

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/zed/zed_node/odom", 10,
            std::bind(&PurePursuitFollower::odomCallback, this, std::placeholders::_1));

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/vo_path", 10,
            std::bind(&PurePursuitFollower::pathCallback, this, std::placeholders::_1));

        autonomous_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/autonomous", 10,
            std::bind(&PurePursuitFollower::autonomousCallback, this, std::placeholders::_1));

        /* ---------- Publisher ---------- */

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead", 10);
        current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(freq_ms_),
            std::bind(&PurePursuitFollower::controlLoop, this));

        RCLCPP_INFO(get_logger(), "VO Pure Pursuit Follower started");
    }

private:
    /* ===================== CALLBACK ===================== */

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg->poses;
        target_idx_ = 0;
        prev_idx_ = 0;
    }

    void autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        autonomous_ = msg->data;
    }

    /* ===================== CONTROL ===================== */

    void controlLoop()
    {
        if (!autonomous_ || path_.empty() || !base_initialized_) return;

        findLookaheadPoint();
        publishLookahead();

        double heading_error = calcHeadingError();
        double w = pid_.cycle(heading_error);

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

    void findLookaheadPoint()
    {
        double lookahead = ld_gain_ * v_max_ + ld_min_;

        for (size_t i = prev_idx_; i < path_.size(); ++i) {
            double dx = path_[i].pose.position.x - current_x_;
            double dy = path_[i].pose.position.y - current_y_;
            double dist = std::hypot(dx, dy);

            if (dist > lookahead) {
                target_idx_ = i;
                prev_idx_ = i;
                return;
            }
        }
        target_idx_ = path_.size();
    }

    double calcHeadingError()
    {
        if (target_idx_ >= path_.size()) return 0.0;

        double dx = path_[target_idx_].pose.position.x - current_x_;
        double dy = path_[target_idx_].pose.position.y - current_y_;

        double target_yaw = std::atan2(dy, dx);
        double err = target_yaw - current_yaw_;

        return std::atan2(std::sin(err), std::cos(err));
    }

    /* ===================== VISUALIZATION ===================== */

    void publishLookahead()
    {
        if (target_idx_ >= path_.size()) return;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = path_[target_idx_].pose.position.x - base_x_;
        pose.pose.position.y = path_[target_idx_].pose.position.y - base_y_;
        lookahead_pub_->publish(pose);
    }

    void publishCurrentPose()
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

    double yawFromQuat(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tfq);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

private:
    /* ---------- ROS ---------- */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* ---------- STATE ---------- */
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t target_idx_{0}, prev_idx_{0};

    double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
    double base_x_{0.0}, base_y_{0.0}, base_yaw_{0.0};

    bool base_initialized_{false};
    bool autonomous_{false};

    /* ---------- PARAM ---------- */
    int freq_ms_;
    double ld_gain_, ld_min_;
    double v_max_, w_max_;

    controller::PositionPid pid_;
};

} 
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
