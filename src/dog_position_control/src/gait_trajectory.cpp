#include "dog_position_control/gait_trajectory.hpp"

#include <algorithm>
#include <cmath>

#include "dog_position_control/math_utils.hpp"

namespace dog_position_control
{

// 摆动相轨迹:
// 采用 cycloid 形式，让抬脚和落脚都比较平滑，同时天然可得到解析速度。
FootState trajectory_cycloid(
  double tau,
  double step_length,
  double step_height,
  double start_x,
  double period)
{
  FootState state;
  state.x = start_x + step_length * (tau - std::sin(2.0 * kPi * tau) / (2.0 * kPi));
  state.y = step_height * (1.0 - std::cos(2.0 * kPi * tau)) / 2.0;

  if (period > 0.0)
  {
    const double dtau = 1.0 / period;
    state.vx = step_length * (1.0 - std::cos(2.0 * kPi * tau)) * dtau;
    state.vy = step_height * kPi * std::sin(2.0 * kPi * tau) * dtau;
  }

  return state;
}

namespace
{

double eased_linear_progress(double tau, double transition_ratio)
{
  tau = clamp01(tau);
  const double blend = std::clamp(transition_ratio, 0.0, 0.45);
  if (blend <= 1e-6)
  {
    return tau;
  }

  if (tau < blend)
  {
    const double u = tau / blend;
    return blend * (2.0 * u * u - u * u * u);
  }

  if (tau > 1.0 - blend)
  {
    const double u = (1.0 - tau) / blend;
    return 1.0 - blend * (2.0 * u * u - u * u * u);
  }

  return tau;
}

double eased_linear_progress_derivative(double tau, double transition_ratio)
{
  tau = clamp01(tau);
  const double blend = std::clamp(transition_ratio, 0.0, 0.45);
  if (blend <= 1e-6)
  {
    return 1.0;
  }

  if (tau < blend)
  {
    const double u = tau / blend;
    return 4.0 * u - 3.0 * u * u;
  }

  if (tau > 1.0 - blend)
  {
    const double u = (1.0 - tau) / blend;
    return 4.0 * u - 3.0 * u * u;
  }

  return 1.0;
}

}  // namespace

// trot 支撑相轨迹:
// 中段保持接近匀速回拉，只在支撑相两端做速度缓入/缓出。
FootState trajectory_stance_trot(
  double tau,
  double step_length,
  double period,
  double transition_ratio)
{
  FootState state;
  const double half_step = step_length * 0.5;
  const double progress = eased_linear_progress(tau, transition_ratio);
  state.x = half_step - step_length * progress;

  if (period > 0.0)
  {
    const double progress_velocity =
      eased_linear_progress_derivative(tau, transition_ratio) / period;
    state.vx = -step_length * progress_velocity;
  }

  return state;
}

}  // namespace dog_position_control
