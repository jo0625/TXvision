#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "armor_tracker/ekf_models.hpp"
#include "armor_tracker/tracker.hpp"

namespace rm_auto_aim
{
namespace
{

using Armor = Tracker::Armor;
using Armors = Tracker::Armors;

Armor makeArmor(double x, double y, double z, double yaw, const std::string & number = "unknown")
{
  Armor armor;
  armor.number = number;
  armor.type = "small";
  armor.distance_to_image_center = 0.0;
  armor.pose.position.x = x;
  armor.pose.position.y = y;
  armor.pose.position.z = z;
  armor.pose.orientation.z = std::sin(yaw / 2.0);
  armor.pose.orientation.w = std::cos(yaw / 2.0);
  return armor;
}

Armors::SharedPtr makeArmors(std::vector<Armor> armors = {})
{
  auto message = std::make_shared<Armors>();
  message->armors = std::move(armors);
  return message;
}

void configureEkf(Tracker & tracker, const std::shared_ptr<double> & dt)
{
  auto f = [dt](const Eigen::VectorXd & x) {
    Eigen::VectorXd next = x;
    next(0) += x(1) * *dt;
    next(2) += x(3) * *dt;
    next(4) += x(5) * *dt;
    next(6) += x(7) * *dt;
    return next;
  };
  auto j_f = [dt](const Eigen::VectorXd &) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(9, 9);
    matrix(0, 1) = *dt;
    matrix(2, 3) = *dt;
    matrix(4, 5) = *dt;
    matrix(6, 7) = *dt;
    return matrix;
  };
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    z << x(0) - x(8) * std::cos(x(6)), x(2) - x(8) * std::sin(x(6)), x(4), x(6);
    return z;
  };
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(4, 9);
    matrix(0, 0) = 1.0;
    matrix(0, 6) = x(8) * std::sin(x(6));
    matrix(0, 8) = -std::cos(x(6));
    matrix(1, 2) = 1.0;
    matrix(1, 6) = -x(8) * std::cos(x(6));
    matrix(1, 8) = -std::sin(x(6));
    matrix(2, 4) = 1.0;
    matrix(3, 6) = 1.0;
    return matrix;
  };
  auto q = [dt](const Eigen::VectorXd &) { return makeProcessNoiseMatrix(*dt, 0.02, 0.1, 0.02); };
  auto r = [](const Eigen::VectorXd & z) {
    return makeMeasurementNoiseMatrix(z, 0.02, 1e-4, 0.02);
  };

  tracker.ekf = ExtendedKalmanFilter{f, h, j_f, j_h, q, r, Eigen::MatrixXd::Identity(9, 9)};
}

class TrackerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    configureEkf(tracker_, dt_);
    tracker_.tracking_thres = 5;
    tracker_.lost_time_thres = 0.3;
  }

  void initialize(const Armor & armor = makeArmor(1.0, 0.0, 0.2, 0.0))
  {
    tracker_.init(makeArmors({armor}));
  }

  void update(const Armors::SharedPtr & message, double dt = 0.05)
  {
    *dt_ = dt;
    tracker_.update(message, dt);
  }

  void enterTracking(const Armor & armor = makeArmor(1.0, 0.0, 0.2, 0.0))
  {
    initialize(armor);
    for (int frame = 2; frame <= tracker_.tracking_thres; ++frame) {
      update(makeArmors({armor}));
    }
    ASSERT_EQ(tracker_.tracker_state, Tracker::TRACKING);
  }

  std::shared_ptr<double> dt_ = std::make_shared<double>(0.05);
  Tracker tracker_{0.5, 1.0};
};

TEST_F(TrackerTest, ThresholdOneTracksOnInitializationFrame)
{
  tracker_.tracking_thres = 1;
  initialize();
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);
}

TEST_F(TrackerTest, FifthConsecutiveFrameEntersTracking)
{
  const auto armor = makeArmor(1.0, 0.0, 0.2, 0.0);
  initialize(armor);
  EXPECT_EQ(tracker_.tracker_state, Tracker::DETECTING);

  for (int frame = 2; frame < tracker_.tracking_thres; ++frame) {
    update(makeArmors({armor}));
    EXPECT_EQ(tracker_.tracker_state, Tracker::DETECTING) << "frame=" << frame;
  }
  update(makeArmors({armor}));
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);
}

TEST_F(TrackerTest, FirstMissAlwaysEntersTempLost)
{
  enterTracking();
  tracker_.lost_time_thres = 0.2;

  update(makeArmors(), 0.05);

  EXPECT_EQ(tracker_.tracker_state, Tracker::TEMP_LOST);
}

TEST_F(TrackerTest, AccumulatedLostDurationEntersLost)
{
  enterTracking();
  tracker_.lost_time_thres = 0.09;

  update(makeArmors(), 0.05);
  EXPECT_EQ(tracker_.tracker_state, Tracker::TEMP_LOST);
  update(makeArmors(), 0.05);
  EXPECT_EQ(tracker_.tracker_state, Tracker::LOST);
}

TEST_F(TrackerTest, RecoveryClearsAccumulatedLostDuration)
{
  const auto armor = makeArmor(1.0, 0.0, 0.2, 0.0);
  enterTracking(armor);
  tracker_.lost_time_thres = 0.09;

  update(makeArmors(), 0.05);
  update(makeArmors({armor}), 0.05);
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);

  update(makeArmors(), 0.05);
  EXPECT_EQ(tracker_.tracker_state, Tracker::TEMP_LOST);
}

TEST_F(TrackerTest, StationaryTargetConvergesToFiniteState)
{
  const auto armor = makeArmor(1.2, -0.3, 0.25, 0.2);
  enterTracking(armor);

  for (int i = 0; i < 100; ++i) {
    update(makeArmors({armor}), 0.02);
  }

  EXPECT_TRUE(tracker_.target_state.allFinite());
  EXPECT_NEAR(tracker_.measurement(0), armor.pose.position.x, 1e-9);
  EXPECT_NEAR(tracker_.measurement(1), armor.pose.position.y, 1e-9);
  EXPECT_NEAR(tracker_.target_state(1), 0.0, 1e-3);
  EXPECT_NEAR(tracker_.target_state(3), 0.0, 1e-3);
}

TEST_F(TrackerTest, ConstantVelocityTargetHasCorrectDirection)
{
  const double dt = 0.05;
  const double velocity = 0.2;
  initialize(makeArmor(1.0, 0.0, 0.2, 0.0));

  for (int i = 1; i <= 60; ++i) {
    update(makeArmors({makeArmor(1.0 + velocity * dt * i, 0.0, 0.2, 0.0)}), dt);
  }

  ASSERT_TRUE(tracker_.target_state.allFinite());
  EXPECT_GT(tracker_.target_state(0), 1.2);
  EXPECT_GT(tracker_.target_state(1), 0.05);
}

TEST_F(TrackerTest, YawRemainsContinuousAcrossPi)
{
  initialize(makeArmor(1.0, 0.0, 0.2, M_PI - 0.03));
  update(makeArmors({makeArmor(1.0, 0.0, 0.2, -M_PI + 0.03)}));

  EXPECT_GT(tracker_.measurement(3), M_PI);
  EXPECT_NEAR(tracker_.measurement(3), M_PI + 0.03, 1e-6);
}

TEST_F(TrackerTest, RadiusClampingPreservesPredictedArmorPosition)
{
  for (double radius : {0.02, 0.8}) {
    SCOPED_TRACE(radius);
    tracker_.reset();
    enterTracking();
    auto state = tracker_.target_state;
    state(1) = 0.3;
    state(3) = -0.2;
    state(6) = 0.7;
    state(7) = 0.1;
    state(8) = radius;
    tracker_.ekf.setState(state);
    const double dt = 0.02;
    const double yaw = state(6) + state(7) * dt;
    const double expected_x = state(0) + state(1) * dt - radius * std::cos(yaw);
    const double expected_y = state(2) + state(3) * dt - radius * std::sin(yaw);

    update(makeArmors(), dt);

    const auto & result = tracker_.target_state;
    EXPECT_DOUBLE_EQ(result(8), radius < 0.12 ? 0.12 : 0.4);
    EXPECT_NEAR(result(0) - result(8) * std::cos(result(6)), expected_x, 1e-12);
    EXPECT_NEAR(result(2) - result(8) * std::sin(result(6)), expected_y, 1e-12);
    EXPECT_DOUBLE_EQ(result(1), state(1));
    EXPECT_DOUBLE_EQ(result(3), state(3));
    EXPECT_DOUBLE_EQ(result(7), state(7));
    EXPECT_TRUE(result.allFinite());
  }
}

TEST(TrackerCandidateYaw, UnknownCandidateOrderDoesNotChangeResult)
{
  const auto initial = makeArmor(1.0, 0.0, 0.2, M_PI - 0.04);
  const auto selected = makeArmor(1.01, 0.0, 0.2, -M_PI + 0.03);
  const auto distractor = makeArmor(1.10, 0.0, 0.2, 0.0);

  auto forward_dt = std::make_shared<double>(0.05);
  Tracker forward(0.5, 1.0);
  configureEkf(forward, forward_dt);
  forward.tracking_thres = 1;
  forward.lost_time_thres = 0.3;
  forward.init(makeArmors({initial}));
  forward.update(makeArmors({selected, distractor}), *forward_dt);

  auto reverse_dt = std::make_shared<double>(0.05);
  Tracker reverse(0.5, 1.0);
  configureEkf(reverse, reverse_dt);
  reverse.tracking_thres = 1;
  reverse.lost_time_thres = 0.3;
  reverse.init(makeArmors({initial}));
  reverse.update(makeArmors({distractor, selected}), *reverse_dt);

  ASSERT_EQ(forward.tracker_state, Tracker::TRACKING);
  ASSERT_EQ(reverse.tracker_state, Tracker::TRACKING);
  EXPECT_TRUE(forward.measurement.isApprox(reverse.measurement, 1e-12));
  EXPECT_TRUE(forward.target_state.isApprox(reverse.target_state, 1e-12));
  EXPECT_GT(forward.measurement(3), M_PI);
}

TEST_F(TrackerTest, ResetAndTargetChangeRestartStateContracts)
{
  enterTracking();
  tracker_.requestTargetChange();
  EXPECT_EQ(tracker_.tracker_state, Tracker::CHANGE_TARGET);

  tracker_.reset();
  EXPECT_EQ(tracker_.tracker_state, Tracker::LOST);

  tracker_.tracking_thres = 1;
  initialize(makeArmor(2.0, 0.0, 0.2, 0.5, "3"));
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);
}

TEST_F(TrackerTest, TargetChangeWhileLostKeepsTrackerUninitialized)
{
  tracker_.reset();
  tracker_.requestTargetChange();

  EXPECT_EQ(tracker_.tracker_state, Tracker::LOST);
}

TEST_F(TrackerTest, SwitchedTargetCountsItsInitializationAsFirstFrame)
{
  enterTracking(makeArmor(1.0, 0.0, 0.2, 0.0, "3"));
  tracker_.tracking_thres = 3;
  tracker_.requestTargetChange();

  const auto replacement = makeArmor(2.0, 0.0, 0.2, 0.4, "4");
  update(makeArmors({replacement}));
  EXPECT_EQ(tracker_.tracker_state, Tracker::DETECTING);
  update(makeArmors({replacement}));
  EXPECT_EQ(tracker_.tracker_state, Tracker::DETECTING);
  update(makeArmors({replacement}));
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);
}

class SingleArmorTest : public testing::Test
{
protected:
  void SetUp() override
  {
    tracker_.ekf = makeSingleArmorFilter([this]() { return dt_; }, 0.1, 0.0, 1e-6);
    tracker_.tracking_thres = 1;
    tracker_.lost_time_thres = 0.3;
    tracker_.init(makeArmors({makeArmor(2.0, 0.1, 0.2, 0.0, "3")}));
  }

  void update(const std::vector<Armor> & armors) { tracker_.update(makeArmors(armors), dt_); }

  double dt_ = 0.02;
  Tracker tracker_{0.15, 1.0, true};
};

TEST_F(SingleArmorTest, OrientationFlipsDoNotMoveStationaryPlateOrLoseTracking)
{
  for (int i = 0; i < 100; ++i) {
    update({makeArmor(2.0, 0.1, 0.2, i % 2 ? M_PI : -M_PI / 2, "3")});
    ASSERT_EQ(tracker_.tracker_state, Tracker::TRACKING);
    EXPECT_NEAR(tracker_.target_state(0), 2.0, 1e-12);
    EXPECT_NEAR(tracker_.target_state(2), 0.1, 1e-12);
    EXPECT_NEAR(tracker_.target_state(4), 0.2, 1e-12);
    EXPECT_TRUE(tracker_.target_state.tail(3).isZero());
  }
  EXPECT_EQ(tracker_.ekf.covariance().rows(), 6);
  EXPECT_EQ(tracker_.tracked_armors_num, ArmorsNum::SINGLE_1);
  EXPECT_DOUBLE_EQ(tracker_.another_r, 0.0);
  EXPECT_DOUBLE_EQ(tracker_.dz, 0.0);
}

TEST_F(SingleArmorTest, LearnsPlateVelocityAndCoastsDuringBriefLoss)
{
  for (int i = 1; i <= 100; ++i) {
    update({makeArmor(
      2.0 + 0.2 * dt_ * i, 0.1 - 0.1 * dt_ * i, 0.2 + 0.05 * dt_ * i, i % 2 ? M_PI : 0.0, "3")});
  }
  ASSERT_EQ(tracker_.tracker_state, Tracker::TRACKING);
  EXPECT_NEAR(tracker_.target_state(1), 0.2, 1e-3);
  EXPECT_NEAR(tracker_.target_state(3), -0.1, 1e-3);
  EXPECT_NEAR(tracker_.target_state(5), 0.05, 1e-3);
  const Eigen::VectorXd previous = tracker_.target_state;
  update({});
  EXPECT_EQ(tracker_.tracker_state, Tracker::TEMP_LOST);
  for (int i = 0; i < 6; i += 2) {
    EXPECT_NEAR(tracker_.target_state(i), previous(i) + previous(i + 1) * dt_, 1e-12);
  }
  for (int i = 0; i < 15; ++i) update({});
  EXPECT_EQ(tracker_.tracker_state, Tracker::LOST);
}

TEST_F(SingleArmorTest, AssociatesNearestSameIdRegardlessOfOrientation)
{
  const auto selected = makeArmor(2.01, 0.1, 0.2, M_PI, "3");
  const auto distractor = makeArmor(2.10, 0.1, 0.2, 0.0, "3");
  const auto different_id = makeArmor(2.0, 0.1, 0.2, 0.0, "4");
  for (const auto & candidates :
       {std::vector<Armor>{selected, distractor, different_id},
        std::vector<Armor>{different_id, distractor, selected}}) {
    tracker_.reset();
    tracker_.init(makeArmors({makeArmor(2.0, 0.1, 0.2, 0.0, "3")}));
    update(candidates);
    ASSERT_EQ(tracker_.tracker_state, Tracker::TRACKING);
    EXPECT_DOUBLE_EQ(tracker_.measurement(0), 2.01);
  }
}

TEST_F(SingleArmorTest, DistantSameIdDoesNotTriggerRotatingArmorJump)
{
  update({makeArmor(3.0, 0.1, 0.2, M_PI, "3")});
  EXPECT_EQ(tracker_.tracker_state, Tracker::TEMP_LOST);
  EXPECT_DOUBLE_EQ(tracker_.target_state(0), 2.0);
  EXPECT_TRUE(tracker_.target_state.tail(3).isZero());
}

TEST_F(SingleArmorTest, TargetChangeAndResetReinitializeSixDimensionalFilter)
{
  tracker_.requestTargetChange();
  auto replacement = makeArmor(3.0, -0.2, 0.4, M_PI, "4");
  replacement.type = "large";
  update({replacement});
  EXPECT_EQ(tracker_.tracked_id, "4");
  EXPECT_EQ(tracker_.tracked_armors_num, ArmorsNum::SINGLE_1);
  EXPECT_DOUBLE_EQ(tracker_.target_state(0), 3.0);
  EXPECT_TRUE(tracker_.ekf.covariance().isIdentity());
  tracker_.reset();
  EXPECT_EQ(tracker_.tracker_state, Tracker::LOST);
  tracker_.init(makeArmors({makeArmor(1.0, 0.0, 0.3, 1.0, "outpost")}));
  update({makeArmor(1.0, 0.0, 0.3, -1.0, "outpost")});
  EXPECT_EQ(tracker_.tracker_state, Tracker::TRACKING);
  EXPECT_EQ(tracker_.tracked_armors_num, ArmorsNum::SINGLE_1);
  EXPECT_DOUBLE_EQ(tracker_.target_state(0), 1.0);
}

}  // namespace
}  // namespace rm_auto_aim
