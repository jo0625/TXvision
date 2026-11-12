#ifndef ARMOR_TRACKER__TIME_UTILS_HPP_
#define ARMOR_TRACKER__TIME_UTILS_HPP_

#include <cmath>

namespace rm_auto_aim
{

inline bool isPositiveFiniteDt(double dt) { return std::isfinite(dt) && dt > 0.0; }

inline bool shouldResetTrackerForDt(double dt, double max_dt)
{
  return !isPositiveFiniteDt(dt) || !isPositiveFiniteDt(max_dt) || dt > max_dt;
}

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__TIME_UTILS_HPP_
