#include "ekf_localizer/mahalanobis.hpp"

#include <cmath>

#include <Eigen/LU>

namespace ekf_localizer
{

double squared_mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance)
{
    return residual.transpose() * covariance.inverse() * residual;
}

double mahalanobis(const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance)
{
    return std::sqrt(squared_mahalanobis(residual, covariance));
}

}
