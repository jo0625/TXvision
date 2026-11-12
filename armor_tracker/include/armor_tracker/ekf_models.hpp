#ifndef ARMOR_TRACKER__EKF_MODELS_HPP_
#define ARMOR_TRACKER__EKF_MODELS_HPP_

#include <Eigen/Dense>

namespace rm_auto_aim
{

Eigen::MatrixXd makeProcessNoiseMatrix(
  double dt, double position_acceleration_variance, double yaw_acceleration_variance,
  double radius_variance);

Eigen::MatrixXd makeMeasurementNoiseMatrix(
  const Eigen::VectorXd & measurement, double position_variance_factor,
  double position_variance_min, double yaw_variance);

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__EKF_MODELS_HPP_
