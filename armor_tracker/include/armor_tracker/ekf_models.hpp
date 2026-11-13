#ifndef ARMOR_TRACKER__EKF_MODELS_HPP_
#define ARMOR_TRACKER__EKF_MODELS_HPP_

#include <Eigen/Dense>
#include <functional>

#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{

Eigen::MatrixXd makeProcessNoiseMatrix(
  double dt, double position_acceleration_variance, double yaw_acceleration_variance,
  double radius_variance);

Eigen::MatrixXd makeMeasurementNoiseMatrix(
  const Eigen::VectorXd & measurement, double position_variance_factor,
  double position_variance_min, double yaw_variance);

// Six states: plate x, vx, y, vy, z, vz. Observations contain position only.
ExtendedKalmanFilter makeSingleArmorFilter(
  const std::function<double()> & timestep, double acceleration_variance,
  double position_variance_factor, double position_variance_min);

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__EKF_MODELS_HPP_
