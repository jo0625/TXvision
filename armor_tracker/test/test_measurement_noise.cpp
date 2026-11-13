#include <gtest/gtest.h>

#include <limits>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "armor_tracker/measurement_noise.hpp"

namespace rm_auto_aim
{

TEST(MeasurementNoise, ZeroFactorUsesMeasuredVarianceAtEveryCoordinate)
{
  const Eigen::Vector4d measurement(2.0, -0.3, 0.0, 0.1);
  const Eigen::MatrixXd r = makeMeasurementNoiseMatrix(measurement, 0.0, 1e-6, 5e-5);
  EXPECT_TRUE(r.isDiagonal());
  EXPECT_TRUE(r.diagonal().head<3>().isConstant(1e-6));
  EXPECT_DOUBLE_EQ(r(3, 3), 5e-5);
}

TEST(MeasurementNoise, ScalingRemainsPositiveAcrossCoordinateZero)
{
  const Eigen::Vector4d measurement(-2.0, 0.0, 1e-6, 0.1);
  const Eigen::MatrixXd r = makeMeasurementNoiseMatrix(measurement, 0.05, 1e-4, 0.02);
  EXPECT_DOUBLE_EQ(r(0, 0), 0.1);
  EXPECT_DOUBLE_EQ(r(1, 1), 1e-4);
  EXPECT_DOUBLE_EQ(r(2, 2), 1e-4);
  EXPECT_DOUBLE_EQ(r(3, 3), 0.02);
}

TEST(MeasurementNoise, RejectsInvalidVarianceFloor)
{
  const Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
  for (double floor :
       {0.0, -1e-6, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(makeMeasurementNoiseMatrix(measurement, 0.0, floor, 5e-5), std::invalid_argument);
  }
}

TEST(MeasurementNoise, RepeatedStaticUpdatesWithZeroFactorRemainFinite)
{
  const auto identity = [](const Eigen::VectorXd & x) { return x; };
  const auto jacobian = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Identity(4, 4); };
  const auto process_noise = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Zero(4, 4); };
  const auto measurement_noise = [](const Eigen::VectorXd & z) {
    return makeMeasurementNoiseMatrix(z, 0.0, 1e-6, 5e-5);
  };
  ExtendedKalmanFilter ekf(
    identity, identity, jacobian, jacobian, process_noise, measurement_noise,
    Eigen::MatrixXd::Identity(4, 4));
  ekf.setState(Eigen::Vector4d::Zero());
  const Eigen::Vector4d measurement(2.0, 0.0, 0.0, 0.1);
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(ekf.predict().allFinite());
    const Eigen::VectorXd state = ekf.update(measurement);
    ASSERT_TRUE(state.allFinite());
    EXPECT_LT((state - measurement).norm(), 1e-5);
  }
}

}  // namespace rm_auto_aim
