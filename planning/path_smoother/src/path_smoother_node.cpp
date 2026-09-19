#include "path_smoother/path_smoother_node.hpp"

#include <tf2_ros/buffer_interface.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <utilities/utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <memory>
#include <utility>

namespace path_smoother {

PathSmootherNode::PathSmootherNode(const rclcpp::NodeOptions& options)
    : PathSmootherNode("", options) {
}

PathSmootherNode::PathSmootherNode(
    const std::string&         name_space,
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("path_smoother_node", name_space, options),
      window_duration_s_(get_parameter("window_duration_s").as_double()),
      tf_timeout_s_(get_parameter("tf_timeout_s").as_double()),
      fit_max_x_m_(get_parameter("fit_max_x_m").as_double()),
      extend_to_x_m_(get_parameter("extend_to_x_m").as_double()),
      output_interval_m_(get_parameter("output_interval_m").as_double()),
      ransac_threshold_m_(get_parameter("ransac_threshold_m").as_double()),
      ransac_iterations_(static_cast<int>(get_parameter("ransac_iterations").as_int())),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_),
      path_subscriber_(this, "/planner/vision_path", rclcpp::QoS(1).keep_last(1).get_rmw_qos_profile()),
      tf_filter_(path_subscriber_, tf_buffer_, "odom", 1, get_node_logging_interface(), get_node_clock_interface(),
                 std::chrono::duration_cast<tf2::Duration>(std::chrono::duration<double>(tf_timeout_s_))) {
    tf_buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_filter_.registerCallback(&PathSmootherNode::path_callback, this);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/global_path", rclcpp::QoS(1).keep_last(1));
}

void PathSmootherNode::path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg) {
    std::vector<Point2D> input;
    input.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
        input.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
    }

    const auto           tf = tf_buffer_.lookupTransform("odom", "base_link", tf2_ros::fromMsg(msg->header.stamp));
    const Pose2D         now{tf.transform.translation.x, tf.transform.translation.y, utils::yaw_from_quaternion(tf.transform.rotation)};
    const rclcpp::Time   stamp(msg->header.stamp);
    window_.emplace_back(stamp, to_odom(input, now));
    while ((stamp - window_.front().first).seconds() > window_duration_s_) {
        window_.pop_front();
    }
    std::vector<std::vector<Point2D>> frames;
    for (const auto& frame : window_) {
        frames.push_back(to_base(frame.second, now));
    }

    std::vector<Point2D> points;
    std::vector<double>  weights;
    for (const auto& frame : frames) {
        std::vector<Point2D> kept;
        for (const auto& point : frame) {
            if (point.x > 0.0 && point.x <= fit_max_x_m_) {
                kept.push_back(point);
            }
        }
        const auto frame_weights = arc_weights(kept);
        points.insert(points.end(), kept.begin(), kept.end());
        weights.insert(weights.end(), frame_weights.begin(), frame_weights.end());
    }
    if (points.size() < 3U) {
        return;
    }

    const auto           inliers = ransac_inliers(points, weights, ransac_threshold_m_, ransac_iterations_);
    std::vector<Point2D> kept;
    std::vector<double>  kept_weights;
    double               x_start = fit_max_x_m_;
    double               x_end   = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (inliers[i]) {
            kept.push_back(points[i]);
            kept_weights.push_back(weights[i]);
            x_start = std::min(x_start, points[i].x);
            x_end   = std::max(x_end, points[i].x);
        }
    }
    const Quadratic c         = fit_quadratic(kept, kept_weights);
    const auto      samples   = sample_quadratic(c, x_start, x_end, output_interval_m_);
    const double    end_yaw   = tangent_yaw(c, samples.back().x);
    const auto      extension = extend_straight(samples.back(), end_yaw, extend_to_x_m_, output_interval_m_);

    nav_msgs::msg::Path output;
    output.header = msg->header;
    output.poses.reserve(samples.size() + extension.size());
    const auto push = [&output](const Point2D& point, const double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header           = output.header;
        pose.pose.position.x  = point.x;
        pose.pose.position.y  = point.y;
        pose.pose.orientation = utils::yaw_to_quaternion(yaw);
        output.poses.push_back(pose);
    };
    for (const auto& point : samples) {
        push(point, tangent_yaw(c, point.x));
    }
    for (const auto& point : extension) {
        push(point, end_yaw);
    }
    path_publisher_->publish(std::make_unique<nav_msgs::msg::Path>(std::move(output)));
}

}  // namespace path_smoother
