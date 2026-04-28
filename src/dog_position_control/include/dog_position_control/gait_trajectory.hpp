#ifndef DOG_POSITION_CONTROL_GAIT_TRAJECTORY_HPP_
#define DOG_POSITION_CONTROL_GAIT_TRAJECTORY_HPP_

#include "dog_position_control/robot_model.hpp"

namespace dog_position_control
{

FootState trajectory_cycloid(
  double tau,
  double step_length,
  double step_height,
  double start_x,
  double period);

FootState trajectory_stance_trot(
  double tau,
  double step_length,
  double period,
  double transition_ratio = 0.15);

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_GAIT_TRAJECTORY_HPP_
