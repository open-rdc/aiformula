#pragma once

#include "vector_e2e_planner/branch_counter.hpp"
#include "vector_e2e_planner/path_builder.hpp"
#include "vector_e2e_planner/vector_e2e_tensorrt.hpp"
#include "vector_e2e_planner/visibility_control.h"

#include <camera_utility/camera_utility.hpp>
#include <rclcpp/rclcpp.hpp>
#include <speed_path_msgs/msg/stop_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vector_e2e_planner {

// 画像から最終経路(odom)と信号停止点を直接出す純 e2e ノード。
// path_smoother と local_planner（ルールの回避）は通さない
class VectorE2ePlannerNode : public rclcpp::Node {
   public:
    VECTOR_E2E_PLANNER_PUBLIC
    explicit VectorE2ePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    VECTOR_E2E_PLANNER_PUBLIC
    explicit VectorE2ePlannerNode(
        const std::string&         name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    std::optional<Pose2D> lookup_odom_pose(const builtin_interfaces::msg::Time& stamp);

    visualization_msgs::msg::MarkerArray::UniquePtr make_marker_array(
        const std::vector<Point2D>& points, const std_msgs::msg::Header& header) const;
    sensor_msgs::msg::Image::UniquePtr make_debug_image(
        const cv::Mat& source, const std::vector<Point2D>& points, const std_msgs::msg::Header& header) const;

    const double      exist_threshold_ = CounterParams().exist_thr;  // 指示スロットの存在判定。カウンタと同じ値
    const bool        blue_;        // 青コース: 分岐計数 + route_plan で経路選択。赤コース: 常に straight + 信号停止
    const double      signal_red_threshold_;
    const double      stop_offset_m_;
    const double      path_valid_threshold_;
    const double      output_interval_m_;
    const double      tf_timeout_s_;

    RoutePlan         route_plan_;
    BranchCounter     counter_;
    PathSmoother      smoother_;
    std::optional<Pose2D> previous_pose_;

    std::unique_ptr<VectorE2eTensorrt>              inference_;
    const tf2::Transform                            base_T_camera_;
    std::optional<camera_utility::CameraIntrinsics> intrinsics_;

    tf2_ros::Buffer                                 tf_buffer_;
    tf2_ros::TransformListener                      tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr      camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           image_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  local_path_publisher_;
    rclcpp::Publisher<speed_path_msgs::msg::StopPoint>::SharedPtr      stop_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              debug_image_publisher_;
};

}  // namespace vector_e2e_planner
