#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

#include "ekf_localizer/mahalanobis.hpp"

using ekf_localizer::mahalanobis;
using ekf_localizer::squared_mahalanobis;

TEST(Mahalanobis, ZeroResidualIsZeroDistance)
{
    Eigen::VectorXd residual(2);
    residual << 0.0, 0.0;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_DOUBLE_EQ(squared_mahalanobis(residual, covariance), 0.0);
    EXPECT_DOUBLE_EQ(mahalanobis(residual, covariance), 0.0);
}

TEST(Mahalanobis, IdentityCovarianceMatchesEuclideanNorm)
{
    Eigen::VectorXd residual(2);
    residual << 3.0, 4.0;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_DOUBLE_EQ(squared_mahalanobis(residual, covariance), 25.0);
    EXPECT_DOUBLE_EQ(mahalanobis(residual, covariance), 5.0);
}

TEST(Mahalanobis, ScalesInverselyWithVariance)
{
    Eigen::VectorXd residual(1);
    residual << 2.0;
    Eigen::MatrixXd covariance(1, 1);
    covariance << 4.0;

    // (2^2) / 4 = 1.0 -> distance 1.0
    EXPECT_DOUBLE_EQ(squared_mahalanobis(residual, covariance), 1.0);
    EXPECT_DOUBLE_EQ(mahalanobis(residual, covariance), 1.0);
}

TEST(Mahalanobis, CorrelatedCovarianceIsHandled)
{
    Eigen::VectorXd residual(2);
    residual << 1.0, 1.0;
    Eigen::MatrixXd covariance(2, 2);
    covariance << 2.0, 0.5,
                  0.5, 1.0;

    const double expected = residual.transpose() * covariance.inverse() * residual;
    EXPECT_DOUBLE_EQ(squared_mahalanobis(residual, covariance), expected);
    EXPECT_DOUBLE_EQ(mahalanobis(residual, covariance), std::sqrt(expected));
}
