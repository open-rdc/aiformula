#include "path_tracker/mpc_node.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_tracker {

MPCNode::MPCNode(const rclcpp::NodeOptions &options)
    : Node("mpc_node", options), dt_(0.1), horizon_(20), max_v_(1.5),
      max_accel_(0.3), max_delta_rate_(1.0), goal_tolerance_(0.1), L_(0.8),
      m_(71.5), I_(7.72) {
  // Parameters
  dt_ = this->declare_parameter("dt", 0.1);
  horizon_ = this->declare_parameter("horizon", 20);
  max_v_ = this->declare_parameter("max_v", 2.0); // Keep for reference
  max_accel_ = this->declare_parameter("max_accel", 2.0);
  max_delta_rate_ = this->declare_parameter("max_delta_rate", 5);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.1);

  // Model Parameters
  L_ = this->declare_parameter("wheelbase", 0.8);
  m_ = this->declare_parameter("mass", 71.5);
  I_ = this->declare_parameter("inertia", 7.72);

  // Subscribers and Publishers
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/e2e_planner/path", 10,
      std::bind(&MPCNode::on_path_received, this, std::placeholders::_1));

  sub_velocity_ =
      this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/vectornav/velocity_body", 10,
          std::bind(&MPCNode::on_velocity_received, this,
                    std::placeholders::_1));

  sub_caster_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/caster_data", 10,
      std::bind(&MPCNode::on_caster_received, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/current_pose", 10,
      std::bind(&MPCNode::on_pose_received, this, std::placeholders::_1));

  pub_cmd_vel_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Timer for control loop (run at 1/dt Hz)
  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&MPCNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "MPC Node (Feedback Integrated) initialized");
}

void MPCNode::on_path_received(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Path received with %zu poses",
              msg->poses.size());
  current_path_ = msg;
}

void MPCNode::on_velocity_received(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  double raw_v = msg->twist.twist.linear.x;
  // Stronger LPF for velocity to kill pulsing (alpha=0.1)
  latest_v_ = 0.9 * latest_v_ + 0.1 * raw_v;
  // Right+ Throughout: IMU is CCW+, so invert to match Right+ model
  latest_w_ = -msg->twist.twist.angular.z;
}

void MPCNode::on_caster_received(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() > 0) {
    // Normalization: Caster sensor appears to match polar convention (CCW+)
    double raw_delta = msg->data[0];
    // Low-pass filter (alpha=0.3) to reduce vibration
    latest_delta_ = 0.7 * latest_delta_ + 0.3 * raw_delta;
  }
}

void MPCNode::on_pose_received(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  latest_pose_ = msg;
}

std::vector<State> MPCNode::transform_path_to_local(
    const nav_msgs::msg::Path::SharedPtr global_path,
    const geometry_msgs::msg::PoseStamped &robot_pose) {
  std::vector<State> local_traj;
  double rx = robot_pose.pose.position.x;
  double ry = robot_pose.pose.position.y;
  // Right+ Throughout: Use raw yaw
  double rtheta = tf2::getYaw(robot_pose.pose.orientation);

  double cos_theta = std::cos(rtheta);
  double sin_theta = std::sin(rtheta);

  for (const auto &p : global_path->poses) {
    double gx = p.pose.position.x;
    double gy = p.pose.position.y;
    // Right+ Throughout: Use raw yaw
    double gtheta = tf2::getYaw(p.pose.orientation);

    // Relative position
    double dx = gx - rx;
    double dy = gy - ry;

    State s;
    // Map to robot frame
    s.x = dx * cos_theta + dy * sin_theta;
    s.y = -dx * sin_theta + dy * cos_theta;
    s.theta = gtheta - rtheta;

    // Normalize angle
    while (s.theta > M_PI)
      s.theta -= 2.0 * M_PI;
    while (s.theta < -M_PI)
      s.theta += 2.0 * M_PI;

    local_traj.push_back(s);
  }
  return local_traj;
}

void MPCNode::control_loop() {
  if (!current_path_ || current_path_->poses.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for path...");
    return;
  }

  if (!latest_pose_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for pose...");
    return;
  }

  // Transform global path to local base_link frame
  std::vector<State> local_path =
      transform_path_to_local(current_path_, *latest_pose_);

  // Use the latest feedback values as the initial state for the MPC solver
  // After coordinate transformation, the robot is always at the local origin
  // (0,0,0)
  State current_state = {0.0, 0.0, 0.0, latest_v_, latest_delta_};

  // Extract reference trajectory from path based on distance (velocity * dt)
  std::vector<State> reference_trajectory;
  double lookahead_dist = 0.0;
  for (int i = 0; i < horizon_; ++i) {
    // target_v = 2.0 to match Bag Cmd
    double target_v_ref = 2.0;
    lookahead_dist += (target_v_ref * dt_) + 0.1;

    // Find the point in local_path closest to lookahead_dist
    size_t best_idx = 0;
    double min_diff = 1e9;
    for (size_t j = 0; j < local_path.size(); ++j) {
      double dist = std::sqrt(local_path[j].x * local_path[j].x +
                              local_path[j].y * local_path[j].y);
      double diff = std::abs(dist - lookahead_dist);
      if (diff < min_diff) {
        min_diff = diff;
        best_idx = j;
      }
    }
    reference_trajectory.push_back(State{local_path[best_idx].x,
                                         local_path[best_idx].y,
                                         local_path[best_idx].theta, 1.4, 0.0});
  }

  // Solve MPC (Kinematic Only)
  std::vector<Control> optimal_controls =
      solve_mpc(current_state, reference_trajectory);

  // Publish command
  if (!optimal_controls.empty()) {
    State next_state = step_model(current_state, optimal_controls[0], dt_);
    double target_v = next_state.v;
    double target_delta = next_state.delta;
    double target_omega = (target_v * std::tan(target_delta)) / L_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_v;
    // Right+ Throughout: omega is Right+, assume actuator/bag/plot are all
    // Right+
    cmd.angular.z = target_omega;
    pub_cmd_vel_->publish(cmd);

    predicted_controls_ = optimal_controls;
    predicted_controls_.erase(predicted_controls_.begin());
    predicted_controls_.push_back(Control{0.0, 0.0});
  }
}

State MPCNode::step_model(const State &s, const Control &u, double dt) {
  State next;
  // Kinematic Bicycle Model
  // x_{k+1} = x_k + v_k * cos(theta_k) * dt
  next.x = s.x + s.v * std::cos(s.theta) * dt;
  // y_{k+1} = y_k + v_k * sin(theta_k) * dt
  next.y = s.y + s.v * std::sin(s.theta) * dt;
  // theta_{k+1} = theta_k + (v_k * tan(delta_k) / L) * dt
  next.theta = s.theta + (s.v * std::tan(s.delta) / L_) * dt;

  // v_{k+1} = v_k + a_k * dt
  next.v = s.v + u.a * dt;
  // delta_{k+1} = delta_k + delta_rate_k * dt
  next.delta = s.delta + u.delta_rate * dt;

  return next;
}

std::vector<Control>
MPCNode::solve_mpc(const State &current_state,
                   const std::vector<State> &reference_trajectory) {
  int N = horizon_;

  // Pack inputs
  double current_state_arr[5] = {current_state.x, current_state.y,
                                 current_state.theta, current_state.v,
                                 current_state.delta};

  std::vector<double> ref_traj_arr(N * 4);
  for (int i = 0; i < N; ++i) {
    ref_traj_arr[i * 4 + 0] = reference_trajectory[i].x;
    ref_traj_arr[i * 4 + 1] = reference_trajectory[i].y;
    ref_traj_arr[i * 4 + 2] = reference_trajectory[i].theta;
    ref_traj_arr[i * 4 + 3] = reference_trajectory[i].v;
  }

  std::vector<double> out_a(N);
  std::vector<double> out_dr(N);

  // Call the ABI-safe bridge function
  solve_mpc_casadi_c(current_state_arr, ref_traj_arr.data(), N, dt_, L_,
                     max_accel_, max_delta_rate_, out_a.data(), out_dr.data());

  // Unpack outputs
  std::vector<Control> optimal_controls;
  for (int i = 0; i < (int)out_a.size(); ++i) {
    optimal_controls.push_back({out_a[i], out_dr[i]});
  }

  if (optimal_controls.empty()) {
    RCLCPP_ERROR(this->get_logger(), "CasADi solver returned empty controls!");
  } else if (std::abs(optimal_controls[0].a) < 1e-9 &&
             std::abs(optimal_controls[0].delta_rate) < 1e-9) {
    RCLCPP_DEBUG(this->get_logger(), "CasADi solver returned zero controls.");
  }

  return optimal_controls;
}

} // namespace path_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_tracker::MPCNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_tracker::MPCNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
