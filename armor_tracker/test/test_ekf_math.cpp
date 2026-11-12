#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "armor_tracker/ekf_models.hpp"
#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
namespace
{

TEST(ExtendedKalmanFilter, JosephUpdateProducesFiniteSymmetricCovariance)
{
  const Eigen::Matrix2d p0 = (Eigen::Matrix2d() << 2.0, 0.7, 0.7, 1.0).finished();
  const Eigen::Matrix2d h_matrix = (Eigen::Matrix2d() << 1.0, 0.2, -0.1, 1.0).finished();
  const Eigen::Matrix2d r_matrix = (Eigen::Matrix2d() << 1e-6, 0.0, 0.0, 2e-6).finished();

  auto identity = [](const Eigen::VectorXd & x) { return x; };
  auto observation = [h_matrix](const Eigen::VectorXd & x) {
    return Eigen::VectorXd(h_matrix * x);
  };
  auto identity_jacobian = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Identity(2, 2); };
  auto observation_jacobian = [h_matrix](const Eigen::VectorXd &) {
    return Eigen::MatrixXd(h_matrix);
  };
  auto zero_q = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Zero(2, 2); };
  auto fixed_r = [r_matrix](const Eigen::VectorXd &) { return Eigen::MatrixXd(r_matrix); };

  ExtendedKalmanFilter ekf{identity, observation, identity_jacobian, observation_jacobian, zero_q,
                           fixed_r,  p0};
  ekf.setState(Eigen::Vector2d(0.0, 0.0));
  ekf.predict();
  const Eigen::Vector2d measurement(1.0, -0.5);
  const Eigen::VectorXd updated = ekf.update(measurement);

  const Eigen::Matrix2d innovation = h_matrix * p0 * h_matrix.transpose() + r_matrix;
  const Eigen::Matrix2d gain =
    innovation.ldlt().solve((p0 * h_matrix.transpose()).transpose()).transpose();
  const Eigen::Matrix2d residual = Eigen::Matrix2d::Identity() - gain * h_matrix;
  const Eigen::Matrix2d expected =
    residual * p0 * residual.transpose() + gain * r_matrix * gain.transpose();
  const Eigen::MatrixXd covariance = ekf.covariance();

  EXPECT_TRUE(updated.allFinite());
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
  EXPECT_TRUE(covariance.isApprox(expected, 1e-12));
  EXPECT_GE(
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(covariance).eigenvalues().minCoeff(), -1e-12);
}

TEST(EkfModels, YawProcessNoiseBlockUsesOneConsistentVariance)
{
  const double dt = 0.2;
  const double position_variance = 3.0;
  const double yaw_variance = 7.0;
  const Eigen::MatrixXd q = makeProcessNoiseMatrix(dt, position_variance, yaw_variance, 11.0);

  EXPECT_DOUBLE_EQ(q(6, 6), std::pow(dt, 4) / 4.0 * yaw_variance);
  EXPECT_DOUBLE_EQ(q(6, 7), std::pow(dt, 3) / 2.0 * yaw_variance);
  EXPECT_DOUBLE_EQ(q(7, 6), std::pow(dt, 3) / 2.0 * yaw_variance);
  EXPECT_DOUBLE_EQ(q(7, 7), std::pow(dt, 2) * yaw_variance);
  EXPECT_NE(q(6, 7), std::pow(dt, 3) / 2.0 * position_variance);
}

TEST(EkfModels, PositionMeasurementVarianceHasStrictPositiveFloorAtZero)
{
  const Eigen::Vector4d measurement(0.0, -0.0, 0.0, 0.0);
  const double minimum_variance = 1e-4;
  const Eigen::MatrixXd r = makeMeasurementNoiseMatrix(measurement, 0.05, minimum_variance, 0.02);

  EXPECT_DOUBLE_EQ(r(0, 0), minimum_variance);
  EXPECT_DOUBLE_EQ(r(1, 1), minimum_variance);
  EXPECT_DOUBLE_EQ(r(2, 2), minimum_variance);
  EXPECT_GT(r.diagonal().head<3>().minCoeff(), 0.0);
}

TEST(EkfModels, PositionMeasurementVarianceRejectsInvalidFloor)
{
  const Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
  EXPECT_THROW(makeMeasurementNoiseMatrix(measurement, 0.05, 0.0, 0.02), std::invalid_argument);
  EXPECT_THROW(
    makeMeasurementNoiseMatrix(measurement, 0.05, std::numeric_limits<double>::quiet_NaN(), 0.02),
    std::invalid_argument);
}

}  // namespace
}  // namespace rm_auto_aim
