// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  P0(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

void ExtendedKalmanFilter::reset(const Eigen::VectorXd & x0)
{
  x_post = x0;
  P_post = P0;
}

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q(x_post);

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;
  P_pri = (0.5 * (P_pri + P_pri.transpose())).eval();

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);

  const Eigen::MatrixXd p_ht = P_pri * H.transpose();
  const Eigen::MatrixXd innovation_covariance = H * p_ht + R;
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation_covariance);
  K = ldlt.solve(p_ht.transpose()).transpose();
  x_post = x_pri + K * (z - h(x_pri));
  const Eigen::MatrixXd residual = I - K * H;
  P_post = residual * P_pri * residual.transpose() + K * R * K.transpose();
  P_post = (0.5 * (P_post + P_post.transpose())).eval();

  return x_post;
}

const Eigen::MatrixXd & ExtendedKalmanFilter::covariance() const { return P_post; }

}  // namespace rm_auto_aim
