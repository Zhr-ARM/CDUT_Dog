#ifndef DOG_POSITION_CONTROL_MATH_UTILS_HPP_
#define DOG_POSITION_CONTROL_MATH_UTILS_HPP_

#include <algorithm>

namespace dog_position_control
{

inline constexpr double kPi = 3.14159265358979323846;

inline double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

inline double smoothstep(double value)
{
  value = clamp01(value);
  return value * value * (3.0 - 2.0 * value);
}

inline double interpolate(double start, double end, double alpha)
{
  return start + (end - start) * clamp01(alpha);
}

inline double slew_limit(double previous, double target, double max_delta)
{
  if (max_delta <= 0.0)
  {
    return previous;
  }

  return std::clamp(target, previous - max_delta, previous + max_delta);
}

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_MATH_UTILS_HPP_
