#ifndef DOG_POSITION_CONTROL_GAIT_SCHEDULE_HPP_
#define DOG_POSITION_CONTROL_GAIT_SCHEDULE_HPP_

#include <array>
#include <vector>

#include "dog_position_control/robot_model.hpp"

namespace dog_position_control
{

// 一条腿在当前步态周期内的相位状态。
//
// local_phase 始终位于 [0, 1)，in_swing 表示当前是否处于摆动相。
// swing_tau / stance_tau 分别是摆动相和支撑相内部的归一化进度。
struct LegPhaseState
{
  double local_phase{0.0};
  bool in_swing{false};
  double swing_tau{0.0};
  double stance_tau{0.0};
  double contact_weight{1.0};
};

double normalize_gait_phase(double phase);

std::array<LegPhaseState, kLegCount> make_gait_schedule(
  double global_phase,
  const std::vector<double> & phase_offsets,
  double swing_ratio);

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_GAIT_SCHEDULE_HPP_
