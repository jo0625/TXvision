// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

enum class ArmorsNum { SINGLE_1 = 1, NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff_, bool single_armor = false);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void update(const Armors::SharedPtr & armors_msg, double dt);

  void reset();

  void requestTargetChange();

  void adaptAngularVelocity(const double & duration);

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  double lost_time_thres;
  int change_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
  } tracker_state;

  std::string tracked_id;
  std::string last_tracked_id;
  Armor tracked_armor;
  ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double dz, another_r;

private:
  void initEKF(const Armor & a);

  void initChange(const Armor & armor_msg);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a, double reference_yaw);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q) const;

  double unwrapYaw(double raw_yaw, double reference_yaw) const;

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  void resetCounters();

  void transitionToLost();

  bool single_armor_;

  double max_match_distance_;

  double max_match_yaw_diff_;

  int detect_count_;
  int change_count_;

  double lost_duration_sec_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
