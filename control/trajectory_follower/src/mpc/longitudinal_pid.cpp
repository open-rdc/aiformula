#include "trajectory_follower/mpc/longitudinal_pid.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{

namespace
{
constexpr double STOP_VELOCITY_EPS = 0.05;  // STOPPED 判定速度 [m/s]
}

void LongitudinalPid::configure(const LongitudinalParams & params)
{
    p_ = params;
    reset();
}

double LongitudinalPid::referenceSpeed(double curvature, double dist_to_end) const
{
    double v = p_.v_max;

    const double kappa = std::abs(curvature);
    if (kappa > 1.0e-6) {
        // 横加速度上限による曲率制限速度 v = sqrt(a_lat_max / |kappa|)
        v = std::min(v, std::sqrt(p_.a_lat_max / kappa));
    }

    // 終端までの停止プロファイル v = sqrt(2 * |a_min| * dist)
    const double a_decel = std::abs(p_.a_min);
    const double v_stop = std::sqrt(std::max(0.0, 2.0 * a_decel * std::max(0.0, dist_to_end)));
    v = std::min(v, v_stop);

    return std::clamp(v, 0.0, p_.v_max);
}

double LongitudinalPid::update(double v_ref, double v_meas)
{
    // STOPPED: 参照速度ほぼ0かつ停止中は速度0を保持し積分をリセット
    if (v_ref <= STOP_VELOCITY_EPS && std::abs(v_meas) <= STOP_VELOCITY_EPS) {
        integral_ = 0.0;
        prev_error_ = 0.0;
        filtered_error_ = 0.0;
        prev_a_cmd_ = 0.0;
        v_cmd_ = 0.0;
        prev_v_ref_ = v_ref;
        initialized_ = true;
        return 0.0;
    }

    const double dt = p_.dt;
    const double error = v_ref - v_meas;

    if (!initialized_) {
        // バンプレス開始: 速度指令を現在速度に合わせる
        v_cmd_ = std::clamp(v_meas, 0.0, p_.v_max);
        filtered_error_ = error;
        prev_error_ = error;
        prev_v_ref_ = v_ref;
    }

    // 速度偏差 LPF
    const double g = std::clamp(p_.lpf_vel_error_gain, 0.0, 1.0);
    filtered_error_ = g * filtered_error_ + (1.0 - g) * error;

    // フィードフォワード: 参照速度の数値微分
    const double a_ff = initialized_ ? (v_ref - prev_v_ref_) / dt : 0.0;

    // フィードバック PID (積分はアンチワインドアップ)
    integral_ += filtered_error_ * dt;
    if (p_.ki > 1.0e-9) {
        const double i_limit = p_.a_max / p_.ki;
        integral_ = std::clamp(integral_, -i_limit, i_limit);
    }
    const double deriv = initialized_ ? (filtered_error_ - prev_error_) / dt : 0.0;
    const double a_fb = p_.kp * filtered_error_ + p_.ki * integral_ + p_.kd * deriv;

    // 加速度クランプ + 加加速度制限
    double a_cmd = std::clamp(a_ff + a_fb, p_.a_min, p_.a_max);
    const double da_max = p_.jerk_max * dt;
    a_cmd = std::clamp(a_cmd, prev_a_cmd_ - da_max, prev_a_cmd_ + da_max);

    // 加速度を速度指令へ積分
    v_cmd_ = std::clamp(v_cmd_ + a_cmd * dt, 0.0, p_.v_max);

    prev_error_ = filtered_error_;
    prev_a_cmd_ = a_cmd;
    prev_v_ref_ = v_ref;
    initialized_ = true;
    return v_cmd_;
}

void LongitudinalPid::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_error_ = 0.0;
    prev_v_ref_ = 0.0;
    prev_a_cmd_ = 0.0;
    v_cmd_ = 0.0;
    initialized_ = false;
}

}  // namespace trajectory_follower
