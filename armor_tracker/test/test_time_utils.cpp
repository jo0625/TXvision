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
