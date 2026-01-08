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
  // パラメータの宣言とデフォルト値の設定
  dt_ = this->declare_parameter("dt", 0.1);
  horizon_ = this->declare_parameter("horizon", 8);
  max_v_ = this->declare_parameter("max_v", 2.0); // 参照用
  max_accel_ = this->declare_parameter("max_accel", 2.0);
  max_delta_rate_ = this->declare_parameter("max_delta_rate", 5);
  velocity_gain_ = this->declare_parameter("velocity_gain", 1.573);
  steer_gain_ = this->declare_parameter("steer_gain", 1.468);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.1);

  // モデルパラメータ
  L_ = this->declare_parameter("wheelbase", 0.8);
  m_ = this->declare_parameter("mass", 71.5);
  I_ = this->declare_parameter("inertia", 7.72);

  // サブスクライバとパブリッシャの設定
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

  pub_cmd_vel_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mpc/estimated_pose", 10);

  // 制御ループタイマーの設定 (1/dt Hz)
  timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_),
                                   std::bind(&MPCNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "MPC Node (Feedback Integrated) initialized");
}

void MPCNode::on_path_received(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Path received with %zu poses, frame: %s",
              msg->poses.size(), msg->header.frame_id.c_str());
  current_path_ = msg;

  bool is_local = (msg->header.frame_id == "base_link" ||
                   msg->header.frame_id == "base_footprint");

  if (!is_local && !is_odom_initialized_ && !msg->poses.empty()) {
    // Pathの開始地点をロボットの初期位置とする (Global Pathの場合)
    double start_x = msg->poses[0].pose.position.x;
    double start_y = msg->poses[0].pose.position.y;
    double start_yaw = tf2::getYaw(msg->poses[0].pose.orientation);

    odom_x_ = start_x;
    odom_y_ = start_y;
    odom_yaw_ = start_yaw;
    last_odom_time_ = this->get_clock()->now();

    is_odom_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Initialized Odometry to Path Start: (%.2f, %.2f, %.2f)",
                start_x, start_y, start_yaw);
  } else if (is_local) {
    // Local PathならOdometry初期化は不要（常に0基準）だが、
    // 速度統合のために時間はリセットしておく
    if (!is_odom_initialized_) {
      last_odom_time_ = this->get_clock()->now();
      is_odom_initialized_ = true; // Mark as initialized so loop runs
      odom_x_ = 0;
      odom_y_ = 0;
      odom_yaw_ = 0;
    }
  }
}

void MPCNode::on_velocity_received(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  double raw_v = msg->twist.twist.linear.x;
  // 線形速度に強力なローパスフィルタを適用 (脈動抑制, alpha=0.1)
  latest_v_ = 0.9 * latest_v_ + 0.1 * raw_v;
  // Right+ Throughout: IMUはCCW+なので反転させてRight+モデルに合わせる
  latest_w_ = -msg->twist.twist.angular.z;

  // Dead Reckoning (Odometry Integration)
  rclcpp::Time current_time = msg->header.stamp;
  if (!is_odom_initialized_) {
    last_odom_time_ = current_time;
    return;
  }

  double dt = (current_time - last_odom_time_).seconds();
  if (dt > 0.0 && dt < 1.0) { // Reject huge jumps or negative dt
    double da = odom_yaw_ + (latest_w_ * dt *
                             0.5); // Runge-Kutta 2nd order approx or midpoint
    odom_x_ += latest_v_ * std::cos(da) * dt;
    odom_y_ += latest_v_ * std::sin(da) * dt;
    odom_yaw_ += latest_w_ * dt;

    // Normalize yaw
    while (odom_yaw_ > M_PI)
      odom_yaw_ -= 2.0 * M_PI;
    while (odom_yaw_ < -M_PI)
      odom_yaw_ += 2.0 * M_PI;
  }
  last_odom_time_ = current_time;
}

void MPCNode::on_caster_received(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() > 0) {
    // キャスター角（ステアリング角）を取得
    double raw_delta = msg->data[0];
    // ローパスフィルタを適用して振動を低減 (alpha=0.3)
    latest_delta_ = 0.7 * latest_delta_ + 0.3 * raw_delta;
  }
}

std::vector<State> MPCNode::transform_path_to_local(
    const nav_msgs::msg::Path::SharedPtr global_path, double rx, double ry,
    double rtheta) {
  std::vector<State> local_traj;

  double cos_theta = std::cos(rtheta);
  double sin_theta = std::sin(rtheta);

  for (const auto &p : global_path->poses) {
    double gx = p.pose.position.x;
    double gy = p.pose.position.y;
    // Right+ Throughout: 生のヨー角を使用
    double gtheta = tf2::getYaw(p.pose.orientation);

    // 相対位置の計算
    double dx = gx - rx;
    double dy = gy - ry;

    State s;
    // ロボット座標系（base_link）へ変換
    s.x = dx * cos_theta + dy * sin_theta;
    s.y = -dx * sin_theta + dy * cos_theta;
    s.theta = gtheta - rtheta;

    // 角度の正規化
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

  if (!is_odom_initialized_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for Odometry Init (Path or Sensor)...");
    return;
  }

  // Publish Internal Odometry State
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    // Use the same frame as the path for consistency
    pose_msg.header.frame_id =
        current_path_->header.frame_id; // "base_link" or "map"/"odom"

    pose_msg.pose.position.x = odom_x_;
    pose_msg.pose.position.y = odom_y_;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    pose_msg.pose.orientation = tf2::toMsg(q);

    pub_estimated_pose_->publish(pose_msg);
  }

  // Use Internal Odometry State
  std::vector<State> local_path;

  bool is_local = (current_path_->header.frame_id == "base_link" ||
                   current_path_->header.frame_id == "base_footprint");

  if (is_local) {
    // already local, just convert type
    for (const auto &p : current_path_->poses) {
      State s;
      s.x = p.pose.position.x;
      s.y = p.pose.position.y;
      s.theta = tf2::getYaw(p.pose.orientation);
      local_path.push_back(s);
    }
  } else {
    local_path =
        transform_path_to_local(current_path_, odom_x_, odom_y_, odom_yaw_);
  }

  // 最新のフィードバック値をMPCソルバーの初期状態として使用
  // 座標変換後はロボットは常に原点 (0,0,0) に位置する
  State current_state = {0.0, 0.0, 0.0, latest_v_, latest_delta_};

  // 走行距離（速度 * dt）に基づいて参照軌道を抽出
  std::vector<State> reference_trajectory;
  double lookahead_dist = 0.0;
  for (int i = 0; i < horizon_; ++i) {
    // 現在の速度に基づいて予測位置を計算（極低速時でも0.5m/s分は先読みする）
    double effective_v = std::max(latest_v_, 0.5);
    lookahead_dist += (effective_v * dt_);

    // local_path の中で lookahead_dist に最も近い点を探す
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
                                         local_path[best_idx].theta, 2.0, 0.0});
  }

  // MPCソルバーを実行 (幾何学モデル版)
  std::vector<Control> optimal_controls =
      solve_mpc(current_state, reference_trajectory);

  // コマンドの公開
  if (!optimal_controls.empty()) {
    // 次の状態をモデルで予測して目標速度/角度を決定
    State next_state = step_model(current_state, optimal_controls[0], dt_);
    double target_v = next_state.v * velocity_gain_; // 速度ゲインを適用
    double target_delta = next_state.delta;
    // ステアリングゲインを適用
    double target_omega =
        ((target_v * std::tan(target_delta)) / L_) * steer_gain_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_v;
    // Right+ Throughout: omegaは右旋回正、現在すべての系で右旋回正として統一
    cmd.angular.z = target_omega;
    pub_cmd_vel_->publish(cmd);

    // 次回の計算のために予測履歴をシフト
    predicted_controls_ = optimal_controls;
    predicted_controls_.erase(predicted_controls_.begin());
    predicted_controls_.push_back(Control{0.0, 0.0});
  }
}

State MPCNode::step_model(const State &s, const Control &u, double dt) {
  State next;
  // 運動学自転車モデル (Kinematic Bicycle Model + Slip Angle)
  // x_{k+1} = x_k + v_k * cos(theta_k + delta_k) * dt
  next.x = s.x + s.v * std::cos(s.theta + s.delta) * dt;
  // y_{k+1} = y_k + v_k * sin(theta_k + delta_k) * dt
  next.y = s.y + s.v * std::sin(s.theta + s.delta) * dt;
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

  // 入力データを配列にパック
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

  // ABI安全なブリッジ関数を介してCasADiソルバーを呼び出し
  solve_mpc_casadi_c(current_state_arr, ref_traj_arr.data(), N, dt_, L_,
                     max_accel_, max_delta_rate_, out_a.data(), out_dr.data());

  // 出力をアンパック
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
