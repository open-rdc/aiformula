#ifndef PATH_TRACKER__MPC_NODE_HPP_
#define PATH_TRACKER__MPC_NODE_HPP_

#include "path_tracker/casadi_solver.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

namespace path_tracker {

struct State {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double delta = 0.0;
};

struct Control {
  double a = 0.0;
  double delta_rate = 0.0;
};

class MPCNode : public rclcpp::Node {
public:
  explicit MPCNode(const rclcpp::NodeOptions &options);

private:
  void on_path_received(const nav_msgs::msg::Path::SharedPtr msg);
  void on_velocity_received(
      const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void
  on_caster_received(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_loop();

  // MPC Solver functions
  std::vector<Control>
  solve_mpc(const State &current_state,
            const std::vector<State> &reference_trajectory);
  State step_model(const State &state, const Control &control, double dt);
  std::vector<State>
  transform_path_to_local(const nav_msgs::msg::Path::SharedPtr global_path,
                          const geometry_msgs::msg::PoseStamped &robot_pose);

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<
      geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_caster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  // パラメータ
  double dt_;
  int horizon_;
  double max_v_;          // 状態拘束などで使用する可能性があるため参照用に保持
  double max_accel_;      // 入力拘束: 最大加速度
  double max_delta_rate_; // 入力拘束: 最大ステアリング速度
  double goal_tolerance_;

  // モデルパラメータ
  double L_; // ホイールベース
  double m_; // 質量
  double I_; // 慣性モーメント

  // 内部状態
  nav_msgs::msg::Path::SharedPtr current_path_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  double latest_v_ = 0.0;
  double latest_w_ = 0.0;
  double latest_delta_ = 0.0;
  std::vector<Control> predicted_controls_; // ウォームスタート用予測履歴
};

} // namespace path_tracker

#endif // PATH_TRACKER__MPC_NODE_HPP_
