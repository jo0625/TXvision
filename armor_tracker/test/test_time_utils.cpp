#include <gtest/gtest.h>

#include <limits>

#include "armor_tracker/time_utils.hpp"

TEST(TimeUtils, AcceptsOnlyPositiveFiniteDt)
{
  EXPECT_TRUE(rm_auto_aim::isPositiveFiniteDt(1.0 / 30.0));
  EXPECT_TRUE(rm_auto_aim::isPositiveFiniteDt(std::numeric_limits<double>::min()));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(0.0));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(-0.0));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(-2.0));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(-std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(rm_auto_aim::isPositiveFiniteDt(std::numeric_limits<double>::quiet_NaN()));
}

TEST(TimeUtils, ResetsForInvalidOrExcessiveTrackerDt)
{
  constexpr double max_dt = 0.2;
  EXPECT_FALSE(rm_auto_aim::shouldResetTrackerForDt(0.05, max_dt));
  EXPECT_FALSE(rm_auto_aim::shouldResetTrackerForDt(max_dt, max_dt));
  EXPECT_TRUE(rm_auto_aim::shouldResetTrackerForDt(0.0, max_dt));
  EXPECT_TRUE(rm_auto_aim::shouldResetTrackerForDt(-0.1, max_dt));
  EXPECT_TRUE(rm_auto_aim::shouldResetTrackerForDt(0.21, max_dt));
  EXPECT_TRUE(
    rm_auto_aim::shouldResetTrackerForDt(std::numeric_limits<double>::infinity(), max_dt));
  EXPECT_TRUE(
    rm_auto_aim::shouldResetTrackerForDt(std::numeric_limits<double>::quiet_NaN(), max_dt));
}
