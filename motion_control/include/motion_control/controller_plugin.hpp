#pragma once

#include <memory>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <object_detection_msgs/msg/object_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>

namespace motion_control
{

// motion_control パッケージは local planner として振る舞い，
// global_path を入力として速度指令値を出力するプラグイン基底クラス．
class ControllerPlugin
{
public:
    using SharedPtr = std::shared_ptr<ControllerPlugin>;
    virtual ~ControllerPlugin() = default;

    virtual void initialize(
        const rclcpp::Logger & logger,
        const rclcpp::Clock::SharedPtr & clock,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) = 0;

    // path: global path (frame_id で示される座標系)
    // ego_pose: map frame の自車位置 (nullptr 可)
    // velocity: body frame の速度フィードバック (nullptr 可)
    // objects: 検出物体 (nullptr 可; 障害物回避未使用のプラグインは無視してよい)
    // target_pose_out: 可視化用の追従ターゲット (frame/stamp はサーバ側で設定)
    virtual std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
        const nav_msgs::msg::Path & path,
        const geometry_msgs::msg::PoseWithCovarianceStamped * ego_pose,
        const geometry_msgs::msg::TwistWithCovarianceStamped * velocity,
        const object_detection_msgs::msg::ObjectInfoArray * objects,
        geometry_msgs::msg::PoseStamped & target_pose_out) = 0;
};

}  // namespace motion_control
