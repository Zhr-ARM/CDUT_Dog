#ifndef ARM_CONTROLLER__KINEMATICS_TYPES_HPP_
#define ARM_CONTROLLER__KINEMATICS_TYPES_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace arm_controller
{

/// @brief 4-DOF robot arm joint vector type.
/// Joint order: base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch.
using JointVector = Eigen::Matrix<double, 4, 1>;

/// @brief Number of joints in the kinematic chain.
constexpr std::size_t kJointCount = 4;

/// @brief IK solver global time budget (50 ms).
constexpr auto kSolverTimeBudget = std::chrono::milliseconds(50);

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

/// Compute full end-effector pose (Isometry3d) from joint angles.
Eigen::Isometry3d compute_forward_kinematics(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset);

/// Compute end-effector translation only.
Eigen::Vector3d compute_end_effector_position(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset);

// ---------------------------------------------------------------------------
// Inverse kinematics (position-only)
// ---------------------------------------------------------------------------

bool solve_inverse_kinematics(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const JointVector & seed,
  double tolerance,
  int max_iterations,
  double damping,
  double max_step,
  JointVector & solution,
  double & final_error,
  int & iterations);

// ---------------------------------------------------------------------------
// Inverse kinematics (position + tool X-axis direction)
// ---------------------------------------------------------------------------

bool solve_inverse_kinematics_with_tool_direction(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_tool_x_direction,
  const JointVector & seed,
  double position_tolerance,
  double direction_tolerance,
  double direction_weight,
  int max_iterations,
  double damping,
  double max_step,
  JointVector & solution,
  double & final_position_error,
  double & final_direction_error,
  int & iterations);

// ---------------------------------------------------------------------------
// Utility functions (inline)
// ---------------------------------------------------------------------------

/// Clamp a vector of joint angles to [lower, upper] limits.
inline JointVector clamp_to_limits(
  const JointVector & joints,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits)
{
  JointVector clamped = joints;
  for (int i = 0; i < clamped.size(); ++i) {
    clamped(i) = std::clamp(
      clamped(i),
      lower_limits[static_cast<std::size_t>(i)],
      upper_limits[static_cast<std::size_t>(i)]);
  }
  return clamped;
}

/// Return the mid-point of each joint's limit range.
inline JointVector mid_limits(
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits)
{
  JointVector joints;
  for (int i = 0; i < joints.size(); ++i) {
    joints(i) = 0.5 * (lower_limits[static_cast<std::size_t>(i)] +
                       upper_limits[static_cast<std::size_t>(i)]);
  }
  return joints;
}

/// Check whether every element of a 4-D vector is finite.
inline bool vector_is_finite(const JointVector & v)
{
  for (int i = 0; i < v.size(); ++i) {
    if (!std::isfinite(v(i))) {
      return false;
    }
  }
  return true;
}

/// Generate a set of 6 diverse seeds for the IK solver.
std::vector<JointVector> make_seed_set(
  const JointVector & seed,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  const Eigen::Vector3d & target_position);

/// Numerical Jacobian (3×4) for position-only tasks.
Eigen::Matrix<double, 3, 4> numerical_jacobian(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const JointVector & joints);

/// 6-D pose+orientation state vector for differencing.
Eigen::Matrix<double, 6, 1> pose_direction_state(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  double direction_weight,
  const JointVector & joints);

/// 6-D task-space error (position + weighted tool-X direction).
Eigen::Matrix<double, 6, 1> pose_direction_error(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_tool_x_direction,
  double direction_weight,
  const JointVector & joints);

/// Augmented numerical Jacobian (6×4) for position + tool-direction tasks.
Eigen::Matrix<double, 6, 4> numerical_pose_direction_jacobian(
  const std::array<Eigen::Vector3d, kJointCount> & joint_origins,
  const std::array<Eigen::Vector3d, kJointCount> & joint_axes,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  double direction_weight,
  const JointVector & joints);

}  // namespace arm_controller

#endif  // ARM_CONTROLLER__KINEMATICS_TYPES_HPP_
