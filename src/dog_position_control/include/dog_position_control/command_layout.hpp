#ifndef DOG_POSITION_CONTROL_COMMAND_LAYOUT_HPP_
#define DOG_POSITION_CONTROL_COMMAND_LAYOUT_HPP_

#include <array>
#include <vector>

#include "dog_position_control/robot_model.hpp"

namespace dog_position_control
{

void write_joint_command(
  std::vector<double> & command_data,
  size_t leg_index,
  size_t joint_index,
  double position,
  double velocity,
  double effort,
  double kp,
  double kd);

void set_leg_joint_values(
  std::array<double, kTotalJointCount> & joint_values,
  const JointIndices & indices,
  const JointTargets & targets);

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_COMMAND_LAYOUT_HPP_
