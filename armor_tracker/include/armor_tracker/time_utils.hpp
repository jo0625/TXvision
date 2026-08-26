#ifndef ARMOR_TRACKER__TIME_UTILS_HPP_
#define ARMOR_TRACKER__TIME_UTILS_HPP_

#include <cmath>

namespace rm_auto_aim
{

inline bool isPositiveFiniteDt(double dt) { return std::isfinite(dt) && dt > 0.0; }

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__TIME_UTILS_HPP_
