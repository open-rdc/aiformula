#include "mpcc_controller/mpcc_controller_plugin.hpp"

#include <cmath>
#include <mutex>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

#include "mpcc_controller/params.hpp"

namespace mpcc_controller {

void MpccControllerPlugin::initialize(
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
  logger_ = logger;
  clock_  = clock;

  const std::string prefix = "mpcc_controller_plugin.";

  // motion model プラグインロード
  const std::string model_plugin =
    params->get_parameter(prefix + "motion_model_plugin").as_string();

  model_loader_ = std::make_shared<pluginlib::ClassLoader<MotionModel>>(
    "mpcc_controller", "mpcc_controller::MotionModel");
  motion_model_ = model_loader_->createSharedInstance(model_plugin);
  motion_model_->initialize(params, prefix);

  // パラメータ読み込み
  const Param             param       = loadParam(params, prefix);
  const CostParam         cost_param  = loadCostParam(params, prefix);
  const BoundsParam       bounds_param = loadBoundsParam(params, prefix);
  const NormalizationParam norm_param  = loadNormalizationParam(params, prefix);

  const int    n_sqp       = static_cast<int>(params->get_parameter(prefix + "n_sqp").as_int());
  const int    n_reset     = static_cast<int>(params->get_parameter(prefix + "n_reset").as_int());
  const double sqp_mixing  = params->get_parameter(prefix + "sqp_mixing").as_double();
  const double control_period = params->get_parameter(prefix + "control_period_s").as_double();
  const double track_w_left  = params->get_parameter(prefix + "track_width_left_m").as_double();
  const double track_w_right = params->get_parameter(prefix + "track_width_right_m").as_double();

  mpc_ = std::make_unique<MPC>(
    n_sqp, n_reset, sqp_mixing, control_period,
    motion_model_, param, cost_param, bounds_param, norm_param,
    track_w_left, track_w_right, logger_);

  RCLCPP_INFO(logger_, "MpccControllerPlugin initialized with model: %s", model_plugin.c_str());
}

std::optional<steered_drive_msg::msg::SteeredDrive> MpccControllerPlugin::computeCommand(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseWithCovarianceStamped * ego_pose,
  geometry_msgs::msg::PoseStamped & target_pose_out)
{
  std::lock_guard<std::mutex> lock(compute_mutex_);

  if (path.poses.empty()) {
    RCLCPP_WARN(logger_, "[computeCommand] empty path");
    return std::nullopt;
  }
  if (!ego_pose) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "[computeCommand] ego_pose required for MPCC but unavailable");
    return std::nullopt;
  }

  // パスが変化したときのみスプラインを再構築
  const auto & fp = path.poses.front().pose.position;
  const bool size_changed  = (path.poses.size() != last_path_size_);
  const bool origin_moved  = last_path_size_ > 0 &&
    std::hypot(fp.x - last_first_x_, fp.y - last_first_y_) > 0.01;

  if (size_changed || origin_moved) {
    RCLCPP_INFO(logger_,
      "[computeCommand] path re-init: size=%zu->%zu, origin_moved=%d "
      "(first=%.3f,%.3f -> %.3f,%.3f)",
      last_path_size_, path.poses.size(), origin_moved,
      last_first_x_, last_first_y_, fp.x, fp.y);
    try {
      mpc_->setPath(path);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "[computeCommand] setPath failed: %s", e.what());
      return std::nullopt;
    }
    last_path_size_ = path.poses.size();
    last_first_x_   = fp.x;
    last_first_y_   = fp.y;
    path_initialized_  = true;
    state_initialized_ = false;
  }

  if (!path_initialized_) return std::nullopt;

  // ego_pose から X, Y, phi を更新（ローカライゼーションフィードバック）
  const double qz = ego_pose->pose.pose.orientation.z;
  const double qw = ego_pose->pose.pose.orientation.w;
  const double phi = 2.0 * std::atan2(qz, qw);

  if (!state_initialized_) {
    latest_state_.setZero();
    state_initialized_ = true;
    RCLCPP_INFO(logger_, "[computeCommand] state re-initialized (setZero)");
  }
  const double s_prev = latest_state_.s;
  latest_state_.X   = ego_pose->pose.pose.position.x;
  latest_state_.Y   = ego_pose->pose.pose.position.y;
  latest_state_.phi = phi;
  // s を現在位置からスプライン投影で更新
  latest_state_.s = mpc_->projectOnSpline(latest_state_);

  RCLCPP_DEBUG(logger_,
    "[computeCommand] state: X=%.3f, Y=%.3f, phi=%.3f, v=%.3f, s=%.3f (ds=%.3f)",
    latest_state_.X, latest_state_.Y, latest_state_.phi,
    latest_state_.v_or_vx, latest_state_.s, latest_state_.s - s_prev);

  const MPCReturn result = mpc_->runMPC(latest_state_);

  RCLCPP_DEBUG(logger_,
    "[computeCommand] u0: du0=%.4f, du1=%.4f, dvs=%.4f",
    result.u0.du0, result.u0.du1, result.u0.dvs);

  // v, vs, ctrl_state は MPC 予測値を次サイクルの初期値として保持
  if (result.mpc_horizon.size() > 1) {
    latest_state_.v_or_vx   = result.mpc_horizon[1].xk.v_or_vx;
    latest_state_.vs        = result.mpc_horizon[1].xk.vs;
    latest_state_.ctrl_state = result.mpc_horizon[1].xk.ctrl_state;
  }

  steered_drive_msg::msg::SteeredDrive cmd;
  cmd.velocity       = latest_state_.v_or_vx;
  cmd.steering_angle = latest_state_.ctrl_state;

  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
    "[computeCommand] s=%.3f (ds=%.3f) | ego=(X=%.3f,Y=%.3f,phi=%.3f) | "
    "state=(v=%.4f,vs=%.4f,ctrl=%.4f) | u0=(du0=%.4f,dvs=%.4f) | "
    "cmd=(vel=%.4f,steer=%.4f)",
    latest_state_.s, latest_state_.s - s_prev,
    ego_pose->pose.pose.position.x, ego_pose->pose.pose.position.y, phi,
    latest_state_.v_or_vx, latest_state_.vs, latest_state_.ctrl_state,
    result.u0.du0, result.u0.dvs,
    cmd.velocity, cmd.steering_angle);

  target_pose_out = path.poses.front();
  return cmd;
}

}  // namespace mpcc_controller

PLUGINLIB_EXPORT_CLASS(
  mpcc_controller::MpccControllerPlugin,
  motion_control::ControllerPlugin)
