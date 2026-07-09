#pragma once

#include <Eigen/Core>

namespace ekf_localizer
{

double squared_mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance);
double mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance);

}
