#include "armor_tracker/ekf_models.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm_auto_aim
{

Eigen::MatrixXd makeProcessNoiseMatrix(
  double dt, double position_acceleration_variance, double yaw_acceleration_variance,
  double radius_variance)
{
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(9, 9);
  const auto set_constant_velocity_block = [&q, dt2, dt3, dt4](
                                             int position_index, double variance) {
    q(position_index, position_index) = dt4 / 4.0 * variance;
    q(position_index, position_index + 1) = dt3 / 2.0 * variance;
    q(position_index + 1, position_index) = dt3 / 2.0 * variance;
    q(position_index + 1, position_index + 1) = dt2 * variance;
  };

  set_constant_velocity_block(0, position_acceleration_variance);
  set_constant_velocity_block(2, position_acceleration_variance);
  set_constant_velocity_block(4, position_acceleration_variance);
  set_constant_velocity_block(6, yaw_acceleration_variance);
  q(8, 8) = dt4 / 4.0 * radius_variance;
  return q;
}

Eigen::MatrixXd makeMeasurementNoiseMatrix(
  const Eigen::VectorXd & measurement, double position_variance_factor,
  double position_variance_min, double yaw_variance)
{
  if (!std::isfinite(position_variance_min) || position_variance_min <= 0.0) {
    throw std::invalid_argument("position measurement variance floor must be finite and positive");
  }

  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(4, 4);
  for (int i = 0; i < 3; ++i) {
    r(i, i) = std::max(std::abs(position_variance_factor * measurement(i)), position_variance_min);
  }
  r(3, 3) = yaw_variance;
  return r;
}

}  // namespace rm_auto_aim
