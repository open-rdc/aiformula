#include "mpcc_controller/spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpcc_controller {

void Spline::genSplines(
  const nav_msgs::msg::Path & path,
  const Param & param,
  double n_left, double n_right)
{
  const int n_pts = static_cast<int>(path.poses.size());
  if (n_pts < 2) {
    throw std::invalid_argument("Path must have at least 2 points");
  }

  max_dist_proj_ = param.max_dist_proj;

  // (1) X, Y 抽出
  std::vector<double> xs(n_pts), ys(n_pts);
  for (int i = 0; i < n_pts; i++) {
    xs[i] = path.poses[i].pose.position.x;
    ys[i] = path.poses[i].pose.position.y;
  }

  // (2) 弧長累積
  std::vector<double> sv(n_pts, 0.0);
  for (int i = 1; i < n_pts; i++) {
    const double dx = xs[i] - xs[i - 1];
    const double dy = ys[i] - ys[i - 1];
    sv[i] = sv[i - 1] + std::hypot(dx, dy);
  }
  length_s_ = sv.back();

  // (3) 均一ステップで再サンプリング
  const int n_resample = std::max(n_pts, 200);
  step_s_ = length_s_ / (n_resample - 1);

  std::vector<double> xs_r(n_resample), ys_r(n_resample);
  std::vector<double> nl_r(n_resample, n_left), nr_r(n_resample, n_right);

  for (int i = 0; i < n_resample; i++) {
    const double s_i = step_s_ * i;
    // 線形補間
    auto it = std::lower_bound(sv.begin(), sv.end(), s_i);
    if (it == sv.end()) --it;
    const int idx = static_cast<int>(std::distance(sv.begin(), it));
    if (idx == 0 || it == sv.begin()) {
      xs_r[i] = xs[0];
      ys_r[i] = ys[0];
    } else {
      const int j = idx;
      const double t = (s_i - sv[j - 1]) / (sv[j] - sv[j - 1]);
      xs_r[i] = xs[j - 1] + t * (xs[j] - xs[j - 1]);
      ys_r[i] = ys[j - 1] + t * (ys[j] - ys[j - 1]);
    }
  }

  // (4) スプライン生成
  spline_x_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
    xs_r.begin(), xs_r.end(), 0.0, step_s_);
  spline_y_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
    ys_r.begin(), ys_r.end(), 0.0, step_s_);
  spline_n_left_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
    nl_r.begin(), nl_r.end(), 0.0, step_s_);
  spline_n_right_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
    nr_r.begin(), nr_r.end(), 0.0, step_s_);

  // (5) 曲率から速度スプライン生成: v = min(v_max, sqrt(a_lat / |kappa|))
  std::vector<double> vv(n_resample);
  for (int i = 0; i < n_resample; i++) {
    const double dx  = spline_x_.prime(step_s_ * i);
    const double dy  = spline_y_.prime(step_s_ * i);
    const double ddx = spline_x_.double_prime(step_s_ * i);
    const double ddy = spline_y_.double_prime(step_s_ * i);
    const double den = std::pow(dx * dx + dy * dy, 1.5);
    const double kappa = (std::abs(den) < 1e-6) ? 0.0 : (dx * ddy - dy * ddx) / den;
    const double kappa_abs = std::max(std::abs(kappa), 1e-4);
    vv[i] = std::min(param.v_max, std::sqrt(param.max_lat_accel / kappa_abs));
  }
  spline_v_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
    vv.begin(), vv.end(), 0.0, step_s_);

  // (6) 投影用データ保持
  path_x_ = Eigen::Map<const Eigen::VectorXd>(xs_r.data(), n_resample);
  path_y_ = Eigen::Map<const Eigen::VectorXd>(ys_r.data(), n_resample);
  path_s_.resize(n_resample);
  for (int i = 0; i < n_resample; i++) path_s_(i) = step_s_ * i;
}

Eigen::Vector2d Spline::getPosition(double s) const
{
  const double su = unwrapInput(s);
  return {spline_x_(su), spline_y_(su)};
}

Eigen::Vector2d Spline::getDerivative(double s) const
{
  const double su = unwrapInput(s);
  return {spline_x_.prime(su), spline_y_.prime(su)};
}

Eigen::Vector2d Spline::getSecondDerivative(double s) const
{
  const double su = unwrapInput(s);
  return {spline_x_.double_prime(su), spline_y_.double_prime(su)};
}

double Spline::getCurvature(double s) const
{
  const double su = unwrapInput(s);
  const double dx  = spline_x_.prime(su);
  const double dy  = spline_y_.prime(su);
  const double ddx = spline_x_.double_prime(su);
  const double ddy = spline_y_.double_prime(su);
  const double den = std::pow(dx * dx + dy * dy, 1.5);
  if (std::abs(den) < 1e-6) return 0.0;
  return (dx * ddy - dy * ddx) / den;
}

double Spline::getNLeft(double s)    const { return spline_n_left_(unwrapInput(s)); }
double Spline::getNRight(double s)   const { return spline_n_right_(unwrapInput(s)); }
double Spline::getVelocity(double s) const { return spline_v_(unwrapInput(s)); }
double Spline::getLength()           const { return length_s_; }

double Spline::projectOnSpline(const State & x) const
{
  const Eigen::Vector2d pos = {x.X, x.Y};
  double s_opt = x.s;
  const Eigen::Vector2d pos_path = {spline_x_(s_opt), spline_y_(s_opt)};

  if ((pos - pos_path).norm() >= max_dist_proj_) {
    // 最近傍点をグローバル探索
    const Eigen::ArrayXd dx = path_x_.array() - pos(0);
    const Eigen::ArrayXd dy = path_y_.array() - pos(1);
    Eigen::ArrayXd dist2 = dx.square() + dy.square();
    Eigen::ArrayXd::Index min_idx;
    dist2.minCoeff(&min_idx);
    s_opt = path_s_(min_idx);
  }

  // Newton法で精密化
  double s_old = s_opt;
  for (int i = 0; i < 20; i++) {
    const double su = unwrapInput(s_opt);
    const Eigen::Vector2d pp  = {spline_x_(su), spline_y_(su)};
    const Eigen::Vector2d dpp = {spline_x_.prime(su), spline_y_.prime(su)};
    const Eigen::Vector2d ddpp = {spline_x_.double_prime(su), spline_y_.double_prime(su)};
    const Eigen::Vector2d diff = pp - pos;
    const double jac     = 2.0 * diff.dot(dpp);
    const double hessian = 2.0 * (dpp.dot(dpp) + diff.dot(ddpp));
    s_opt -= jac / hessian;
    s_opt = unwrapInput(s_opt);
    if (std::abs(s_old - s_opt) <= 1e-5) return s_opt;
    s_old = s_opt;
  }
  return s_opt;
}

double Spline::unwrapInput(double s) const
{
  return s - length_s_ * std::floor(s / length_s_);
}

}  // namespace mpcc_controller
