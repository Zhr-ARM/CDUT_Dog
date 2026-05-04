#include "dog_position_control/gait_schedule.hpp"

#include <algorithm>
#include <cmath>

#include "dog_position_control/math_utils.hpp"

namespace dog_position_control
{

double normalize_gait_phase(double phase)
{
  phase = std::fmod(phase, 1.0);
  if (phase < 0.0)
  {
    phase += 1.0;
  }
  return phase;
}

std::array<LegPhaseState, kLegCount> make_gait_schedule(
  double global_phase,
  const std::vector<double> & phase_offsets,
  double swing_ratio)
{
  std::array<LegPhaseState, kLegCount> schedule{};
  const double safe_swing_ratio = std::clamp(swing_ratio, 1e-6, 1.0 - 1e-6);
  const double stance_ratio = 1.0 - safe_swing_ratio;

  for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
  {
    const double phase_offset =
      leg_index < phase_offsets.size() ? phase_offsets[leg_index] : 0.0;
    LegPhaseState state;
    state.local_phase = normalize_gait_phase(global_phase + phase_offset);
    state.in_swing = state.local_phase < safe_swing_ratio;
    if (state.in_swing)
    {
      state.swing_tau = clamp01(state.local_phase / safe_swing_ratio);
      state.stance_tau = 0.0;
      state.contact_weight = 0.0;
    }
    else
    {
      state.swing_tau = 1.0;
      state.stance_tau = clamp01((state.local_phase - safe_swing_ratio) / stance_ratio);
      state.contact_weight = 1.0;
    }

    schedule[leg_index] = state;
  }

  return schedule;
}

}  // namespace dog_position_control
