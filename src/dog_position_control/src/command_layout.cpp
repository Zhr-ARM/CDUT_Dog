#include "dog_position_control/command_layout.hpp"

namespace dog_position_control
{

// 将单个关节命令写入整机扁平数组。
//
// 数组布局固定为:
//   leg_0 joint_0 [p, v, t_ff, kp, kd]
//   leg_0 joint_1 [p, v, t_ff, kp, kd]
//   ...
//   leg_3 joint_2 [p, v, t_ff, kp, kd]
void write_joint_command(
  std::vector<double> & command_data,
  size_t leg_index,
  size_t joint_index,
  double position,
  double velocity,
  double effort,
  double kp,
  double kd)
{
  const size_t base = leg_index * kLegCommandSize + joint_index * kValuesPerJoint;
  command_data[base + 0] = position;
  command_data[base + 1] = velocity;
  command_data[base + 2] = effort;
  command_data[base + 3] = kp;
  command_data[base + 4] = kd;
}

void set_leg_joint_values(
  std::array<double, kTotalJointCount> & joint_values,
  const JointIndices & indices,
  const JointTargets & targets)
{
  joint_values[indices.haa] = targets.haa;
  joint_values[indices.hfe] = targets.hfe;
  joint_values[indices.kfe] = targets.kfe;
}

}  // namespace dog_position_control
