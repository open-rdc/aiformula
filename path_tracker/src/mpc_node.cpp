#include "path_tracker/mpc_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace path_tracker {

MPCNode::MPCNode(const rclcpp::NodeOptions &options)
    : Node("mpc_node", options), dt_(0.1), horizon_(20), max_v_(1.0),
      max_w_(1.0), goal_tolerance_(0.1) {
  // Parameters
  dt_ = this->declare_parameter("dt", 0.1);
  horizon_ = this->declare_parameter("horizon", 20);
  max_v_ = this->declare_parameter("max_v", 1.0);
  max_w_ = this->declare_parameter("max_w", 1.0);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.1);

  // Subscribers and Publishers
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "e2e_planner/path", 10,
      std::bind(&MPCNode::on_path_received, this, std::placeholders::_1));

  pub_cmd_vel_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Timer for control loop (run at 1/dt Hz)
  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&MPCNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "MPC Node initialized");
}

void MPCNode::on_path_received(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Path received with %zu poses", msg->poses.size());
  current_path_ = msg;
}

void MPCNode::control_loop() {
  if (!current_path_ || current_path_->poses.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for path...");
    return;
  }

  // In a real scenario, we should get the current robot state from TF or
  // Odometry. For simplicity here, we assume the path starts at the robot's
  // current location OR we are just tracking the path relative to the robot
  // (local path). However, usually 'e2e_planner/path' is global. Let's assume
  // the first point of the path is close to the robot for now, or better, we
  // should assume the robot is at (0,0,0) in the "base_link" frame and we
  // transform the path to "base_link". BUT, the user's pure_pursuit code does:
  // "Select a lookahead pose measured in the robot base frame."
  // This implies the path is already in the robot frame OR they are doing TF
  // lookups. Wait, the pure_pursuit code checks `pose.pose.position.x`
  // directly. If the path is in `map` frame, this is wrong unless the robot is
  // at (0,0). Let's check pure_pursuit again. It calculates `hypot(dx, dy)`. If
  // path is global, this is distance from origin (0,0). This strongly suggests
  // the path is in the ROBOT FRAME (local path). So I will assume the robot is
  // at (0,0,0).

  State current_state = {0.0, 0.0, 0.0};

  // Extract reference trajectory from path
  // We need to resample or select points for the horizon.
  std::vector<State> reference_trajectory;

  // Simple strategy: take the next 'horizon' points from the path.
  // Assuming path density is roughly consistent with dt * v_ref.
  // If not, we should interpolate. For now, just take points.

  size_t path_idx = 0;
  for (int i = 0; i < horizon_; ++i) {
    if (path_idx < current_path_->poses.size()) {
      const auto &pose = current_path_->poses[path_idx].pose;
      double yaw = tf2::getYaw(pose.orientation);
      reference_trajectory.push_back({pose.position.x, pose.position.y, yaw});
      path_idx++; // In real impl, advance based on distance
    } else {
      // Replicate last point
      reference_trajectory.push_back(reference_trajectory.back());
    }
  }

  // Solve MPC
  std::vector<Control> optimal_controls =
      solve_mpc(current_state, reference_trajectory);

  // Publish first control
  if (!optimal_controls.empty()) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = optimal_controls[0].v;
    cmd.angular.z = optimal_controls[0].w;
    pub_cmd_vel_->publish(cmd);

    // Update warm start
    predicted_controls_ = optimal_controls;
    predicted_controls_.erase(predicted_controls_.begin());
    predicted_controls_.push_back(Control{0.0, 0.0});
  }
}

State MPCNode::step_model(const State &state, const Control &control,
                          double dt) {
  State next_state;
  next_state.x = state.x + control.v * cos(state.theta) * dt;
  next_state.y = state.y + control.v * sin(state.theta) * dt;
  next_state.theta = state.theta + control.w * dt;
  return next_state;
}

std::vector<Control>
MPCNode::solve_mpc(const State &current_state,
                   const std::vector<State> &reference_trajectory) {
  // Simple Gradient Descent MPC
  // Initialize controls
  if (predicted_controls_.size() != (size_t)horizon_) {
    predicted_controls_.assign(horizon_, {0.0, 0.0});
  }

  std::vector<Control> controls = predicted_controls_;
  int max_iter = 10;
  double learning_rate = 0.1; // Tunable

  for (int iter = 0; iter < max_iter; ++iter) {
    std::vector<State> predicted_states;
    predicted_states.push_back(current_state);

    // Forward pass
    for (int i = 0; i < horizon_; ++i) {
      predicted_states.push_back(
          step_model(predicted_states.back(), controls[i], dt_));
    }

    // Backward pass / Gradient calculation (Finite Difference for simplicity)
    // Perturb each control input and see effect on cost.
    // Note: This is O(N*2) per iteration, which is slow.
    // For N=20, it's 40 perturbations. 40 * 20 steps = 800 steps.
    // 10 iters = 8000 steps. Fast enough for C++.

    for (int i = 0; i < horizon_; ++i) {
      // Perturb v
      double epsilon = 1e-3;
      Control perturbed_v = controls[i];
      perturbed_v.v += epsilon;

      // Compute cost diff
      // We only need to re-simulate from step i to end.
      // But for simplicity, let's just compute gradient for this single step's
      // impact? No, changing u[i] affects all x[k] for k > i. Let's implement a
      // simpler cost gradient: dJ/du_i = sum_{k=i+1}^N (dJ/dx_k * dx_k/du_i) +
      // dJ/du_i_direct

      // Let's stick to simple random sampling or just simple P-controller
      // guided descent? Actually, for this task, maybe just a simple sampling
      // based method (MPPI style) is easier? No, let's do a very simple
      // gradient descent with finite diff.

      // Calculate original cost
      double original_cost = 0.0;
      {
        State temp_state = current_state;
        for (int k = 0; k < horizon_; ++k) {
          temp_state = step_model(temp_state, controls[k], dt_);
          double dx = temp_state.x - reference_trajectory[k].x;
          double dy = temp_state.y - reference_trajectory[k].y;
          double dtheta = temp_state.theta - reference_trajectory[k].theta;
          original_cost += dx * dx + dy * dy + 0.1 * dtheta * dtheta;
        }
      }

      // Perturb V
      controls[i].v += epsilon;
      double v_cost = 0.0;
      {
        State temp_state = current_state;
        for (int k = 0; k < horizon_; ++k) {
          temp_state = step_model(temp_state, controls[k], dt_);
          double dx = temp_state.x - reference_trajectory[k].x;
          double dy = temp_state.y - reference_trajectory[k].y;
          double dtheta = temp_state.theta - reference_trajectory[k].theta;
          v_cost += dx * dx + dy * dy + 0.1 * dtheta * dtheta;
        }
      }
      double grad_v = (v_cost - original_cost) / epsilon;
      controls[i].v -= epsilon; // Restore

      // Perturb W
      controls[i].w += epsilon;
      double w_cost = 0.0;
      {
        State temp_state = current_state;
        for (int k = 0; k < horizon_; ++k) {
          temp_state = step_model(temp_state, controls[k], dt_);
          double dx = temp_state.x - reference_trajectory[k].x;
          double dy = temp_state.y - reference_trajectory[k].y;
          double dtheta = temp_state.theta - reference_trajectory[k].theta;
          w_cost += dx * dx + dy * dy + 0.1 * dtheta * dtheta;
        }
      }
      double grad_w = (w_cost - original_cost) / epsilon;
      controls[i].w -= epsilon; // Restore

      // Update
      controls[i].v -= learning_rate * grad_v;
      controls[i].w -= learning_rate * grad_w;

      // Clamp
      controls[i].v =
          std::clamp(controls[i].v, 0.0, max_v_); // Assume forward only
      controls[i].w = std::clamp(controls[i].w, -max_w_, max_w_);
    }
  }

  return controls;
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
