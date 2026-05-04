#ifndef DOG_POSITION_CONTROL_LEG_KINEMATICS_HPP_
#define DOG_POSITION_CONTROL_LEG_KINEMATICS_HPP_

#include "dog_position_control/robot_model.hpp"

namespace dog_position_control
{

IKResult inverse_kinematics(const LegGeometry & geometry, double x, double y);
LegPose2D forward_kinematics(const LegGeometry & geometry, double hfe, double kfe);
JointVel compute_joint_velocity(
  const LegGeometry & geometry,
  double x,
  double y,
  double vx,
  double vy);

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_LEG_KINEMATICS_HPP_
