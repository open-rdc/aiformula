#pragma once

#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <vector>

#include "mpcc_controller/params.hpp"
#include "mpcc_controller/types.hpp"

namespace mpcc_controller {

class Spline
{
public:
  // nav_msgs::msg::Path から弧長パラメータ化スプラインを生成
  void genSplines(const nav_msgs::msg::Path & path, const Param & param,
                  double n_left, double n_right);

  Eigen::Vector2d getPosition(double s)         const;
  Eigen::Vector2d getDerivative(double s)       const;
  Eigen::Vector2d getSecondDerivative(double s) const;
  double getCurvature(double s)                 const;
  double getNLeft(double s)                     const;
  double getNRight(double s)                    const;
  double getVelocity(double s)                  const;
  double getLength()                            const;
  double projectOnSpline(const State & x)       const;

private:
  double unwrapInput(double s) const;

  boost::math::interpolators::cardinal_cubic_b_spline<double> spline_x_;
  boost::math::interpolators::cardinal_cubic_b_spline<double> spline_y_;
  boost::math::interpolators::cardinal_cubic_b_spline<double> spline_n_left_;
  boost::math::interpolators::cardinal_cubic_b_spline<double> spline_n_right_;
  boost::math::interpolators::cardinal_cubic_b_spline<double> spline_v_;

  double length_s_  = 0.0;
  double step_s_    = 0.0;
  double max_dist_proj_ = 5.0;

  // 投影用データ保持
  Eigen::VectorXd path_x_;
  Eigen::VectorXd path_y_;
  Eigen::VectorXd path_s_;
};

}  // namespace mpcc_controller
