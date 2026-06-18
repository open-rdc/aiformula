#pragma once

namespace trajectory_follower
{

// 縦(並進)方向の速度制御パラメータ。
// Autoware pid_longitudinal_controller の FF + PID + 加加速度制限 + 状態機械を
// 速度指令型インターフェース向けに適合した構成。
struct LongitudinalParams
{
    double v_max = 2.0;              // 最大速度 [m/s]
    double a_lat_max = 2.0;          // 横加速度上限 [m/s^2] (曲率制限速度の算出に使用)
    double a_max = 1.0;              // 加速上限 [m/s^2]
    double a_min = -2.0;             // 減速上限 [m/s^2] (負値)
    double jerk_max = 2.0;           // 加加速度上限 [m/s^3]
    double kp = 0.8;
    double ki = 0.1;
    double kd = 0.0;
    double lpf_vel_error_gain = 0.9; // 速度偏差LPFゲイン (1に近いほど平滑)
    double stop_distance = 0.5;      // STOPPING へ入る終端距離 [m]
    double dt = 0.05;                // 制御周期 [s]
};

// 参照速度生成 + 速度偏差PID + 加加速度制限 + 停止制御を行う ROS 非依存クラス。
// 出力は速度指令 [m/s]。加速度ドメインでPIDを計算し速度へ積分して指令を作る。
class LongitudinalPid
{
public:
    void configure(const LongitudinalParams & params);

    // 経路曲率 |curvature| と終端までの距離から参照速度 [m/s] を生成する。
    //   v = min(v_max, sqrt(a_lat_max/|curvature|), sqrt(2*|a_min|*dist_to_end))
    double referenceSpeed(double curvature, double dist_to_end) const;

    // 参照速度 v_ref と現在速度 v_meas から次の速度指令 [m/s] を計算する。
    double update(double v_ref, double v_meas);

    // 積分項・履歴をリセットする (自律解除/停止時)。
    void reset();

    double commandedVelocity() const { return v_cmd_; }
    double commandedAcceleration() const { return prev_a_cmd_; }
    double integral() const { return integral_; }

private:
    LongitudinalParams p_{};
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double filtered_error_ = 0.0;
    double prev_v_ref_ = 0.0;
    double prev_a_cmd_ = 0.0;
    double v_cmd_ = 0.0;
    bool initialized_ = false;
};

}  // namespace trajectory_follower
