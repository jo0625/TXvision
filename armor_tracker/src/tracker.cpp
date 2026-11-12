// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <tuple>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff_)
: tracking_thres(5),
  lost_time_thres(0.3),
  change_thres(20),
  tracker_state(LOST),
  tracked_id(std::string("")),
  last_tracked_id(std::string("")),
  tracked_armors_num(ArmorsNum::NORMAL_4),
  info_position_diff(DBL_MAX),
  info_yaw_diff(DBL_MAX),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  dz(0.0),
  another_r(0.2),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff_),
  detect_count_(0),
  change_count_(0),
  lost_duration_sec_(0.0),
  last_yaw_(0.0)
{
}

void Tracker::reset()
{
  tracker_state = LOST;
  tracked_id.clear();
  last_tracked_id.clear();
  measurement.setZero();
  target_state.setZero();
  resetCounters();
  last_yaw_ = 0.0;
}

void Tracker::requestTargetChange()
{
  if (tracker_state == LOST) {
    resetCounters();
    last_yaw_ = 0.0;
    return;
  }
  tracker_state = CHANGE_TARGET;
  resetCounters();
  last_yaw_ = 0.0;
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = tracked_armor.number;
  last_tracked_id = tracked_id;
  resetCounters();
  detect_count_ = 1;
  tracker_state = tracking_thres <= 1 ? TRACKING : DETECTING;

  updateArmorsNum(tracked_armor);
}

void Tracker::initChange(const Armor & armor_msg)
{
  initEKF(armor_msg);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = armor_msg.number;
  last_tracked_id = tracked_id;
  resetCounters();
  detect_count_ = 1;
  tracker_state = tracking_thres <= 1 ? TRACKING : DETECTING;

  updateArmorsNum(armor_msg);
}

void Tracker::update(const Armors::SharedPtr & armors_msg, double dt)
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    // Find the closest armor with the same id
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    double measured_yaw = 0.0;
    std::tuple<double, double, double, double, double, double> best_same_id_score{
      DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
    int diff_count = 0;
    Armor diff_tracked_armor;
    std::tuple<double, double, double, double> best_diff_id_score{
      DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX};
    for (const auto & armor : armors_msg->armors) {
      // Only consider armors with the same id
      if (armor.number == tracked_id) {
        same_id_armors_count++;
        // Calculate the difference between the predicted position and the current armor position
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        const double raw_yaw = orientationToYaw(armor.pose.orientation);
        const double candidate_yaw = unwrapYaw(raw_yaw, ekf_prediction(6));
        const double candidate_yaw_diff = std::abs(candidate_yaw - ekf_prediction(6));
        const auto score =
          std::make_tuple(position_diff, candidate_yaw_diff, p.x, p.y, p.z, candidate_yaw);
        if (score < best_same_id_score) {
          best_same_id_score = score;
          min_position_diff = position_diff;
          yaw_diff = candidate_yaw_diff;
          measured_yaw = candidate_yaw;
          tracked_armor = armor;
          same_id_armor = armor;
        }
      } else if (tracker_state == CHANGE_TARGET) {
        // Count diff_armor
        diff_count += 1;
        // Calculate the difference between the predicted position and the current armor position
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        const auto score = std::make_tuple(position_diff, p.x, p.y, p.z);
        if (score < best_diff_id_score) {
          best_diff_id_score = score;
          diff_tracked_armor = armor;
        }
      }
    }
    if (diff_count != 0) {
      initChange(diff_tracked_armor);
      return;
    }

    // Store tracker info
    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    // Check if the distance and yaw difference of closest armor are within the threshold
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      // Matched armor found
      matched = true;
      auto p = tracked_armor.pose.position;
      // Update EKF
      last_yaw_ = measured_yaw;
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      // Matched armor not found, but there is only one armor with the same id
      // and yaw has jumped, take this case as the target is spinning and armor jumped
      handleArmorJump(same_id_armor, ekf_prediction(6));
    } else {
      // No matched armor found
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
    }
  }

  // Keep the observed armor position unchanged when constraining the radius.
  if (target_state(8) < 0.12 || target_state(8) > 0.4) {
    const double radius = target_state(8) < 0.12 ? 0.12 : 0.4;
    const double radius_change = radius - target_state(8);
    target_state(0) += radius_change * std::cos(target_state(6));
    target_state(2) += radius_change * std::sin(target_state(6));
    target_state(8) = radius;
    ekf.setState(target_state);
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ >= tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      transitionToLost();
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_duration_sec_ = dt;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_duration_sec_ += dt;
      if (lost_duration_sec_ >= lost_time_thres) {
        transitionToLost();
      }
    } else {
      tracker_state = TRACKING;
      detect_count_ = 0;
      lost_duration_sec_ = 0.0;
      change_count_ = 0;
    }
  } else if (tracker_state == CHANGE_TARGET) {
    change_count_++;
    if (change_count_ >= change_thres) {
      tracker_state = TRACKING;
      change_count_ = 0;
    }
  }
}

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  const double yaw = orientationToYaw(a.pose.orientation);
  last_yaw_ = yaw;

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.2;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf.reset(target_state);
}

void Tracker::updateArmorsNum(const Armor & armor)
{
  if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::handleArmorJump(const Armor & current_armor, double reference_yaw)
{
  const double raw_yaw = orientationToYaw(current_armor.pose.orientation);
  const double yaw = unwrapYaw(raw_yaw, reference_yaw);
  last_yaw_ = yaw;
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  // Only 4 armors has 2 radius and height
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    dz = target_state(4) - current_armor.pose.position.z;
    target_state(4) = current_armor.pose.position.z;
    std::swap(target_state(8), another_r);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // If position difference is larger than max_match_distance_,
  // take this case as the ekf diverged, reset the state
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // za
    target_state(5) = 0;                   // vza
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q) const
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double Tracker::unwrapYaw(double raw_yaw, double reference_yaw) const
{
  return reference_yaw + angles::shortest_angular_distance(reference_yaw, raw_yaw);
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

void Tracker::resetCounters()
{
  detect_count_ = 0;
  change_count_ = 0;
  lost_duration_sec_ = 0.0;
}

void Tracker::transitionToLost()
{
  tracker_state = LOST;
  resetCounters();
  last_yaw_ = 0.0;
}

}  // namespace rm_auto_aim
