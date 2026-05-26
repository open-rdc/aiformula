#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>

#include <motion_control/controller_plugin.hpp>

#include "mpcc_controller/motion_model.hpp"
#include "mpcc_controller/mpc.hpp"
#include "mpcc_controller/types.hpp"
#include "mpcc_controller/visibility_control.h"

namespace mpcc_controller {

class MPCC_CONTROLLER_PUBLIC MpccControllerPlugin : public motion_control::ControllerPlugin
{
public:
  void initialize(
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params) override;

  std::optional<steered_drive_msg::msg::SteeredDrive> computeCommand(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseWithCovarianceStamped * ego_pose,
    geometry_msgs::msg::PoseStamped & target_pose_out) override;

private:
  std::shared_ptr<pluginlib::ClassLoader<MotionModel>> model_loader_;
  std::shared_ptr<MotionModel>                         motion_model_;
  std::unique_ptr<MPC>                                 mpc_;

  State  latest_state_;
  bool   state_initialized_ = false;
  bool   path_initialized_  = false;
  size_t last_path_size_    = 0;
  double last_first_x_      = 0.0;
  double last_first_y_      = 0.0;

  mutable std::mutex        compute_mutex_;

  rclcpp::Logger            logger_{rclcpp::get_logger("mpcc_controller_plugin")};
  rclcpp::Clock::SharedPtr  clock_;
};

}  // namespace mpcc_controller
