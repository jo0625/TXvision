#ifndef ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_
#define ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm_auto_aim
{

// Position diagonal entries are variances in m^2; yaw variance is in rad^2.
inline Eigen::MatrixXd makeMeasurementNoiseMatrix(
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

#endif  // ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_
