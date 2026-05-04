#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace arm_controller
{

using JointVector = Eigen::Matrix<double, 4, 1>;

Eigen::Vector3d compute_end_effector_position(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset);

Eigen::Isometry3d compute_forward_kinematics(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset);

namespace
{

/**
 * @brief 将关节角度值钳位到指定的上下限范围内
 * 
 * 对每个关节角度逐一应用钳位操作，确保其值不低于下限且不高于上限。
 * 使用 std::clamp 实现，保证输出值在 [lower_limits[i], upper_limits[i]] 区间内。
 * 
 * @param joints 输入的关节角度向量（4维）
 * @param lower_limits 各关节的角度下限数组（弧度）
 * @param upper_limits 各关节的角度上限数组（弧度）
 * 
 * @return JointVector 钳位后的关节角度向量
 */
JointVector clamp_to_limits(
  const JointVector & joints,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits)
{
  JointVector clamped = joints;
  for (int index = 0; index < clamped.size(); ++index) {
    clamped(index) = std::clamp(
      clamped(index),
      lower_limits[static_cast<std::size_t>(index)],
      upper_limits[static_cast<std::size_t>(index)]);
  }
  return clamped;
}

/**
 * @brief 计算各关节角度上下限的中间值
 * 
 * 对每个关节计算 (lower + upper) / 2，生成一个位于关节行程中点的关节向量。
 * 该函数常用于为逆运动学求解器生成初始种子点，提供一个安全的初始猜测值。
 * 
 * @param lower_limits 各关节的角度下限数组（弧度）
 * @param upper_limits 各关节的角度上限数组（弧度）
 * 
 * @return JointVector 各关节角度上下限的中间值向量
 */
JointVector mid_limits(
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits)
{
  JointVector joints;
  for (int index = 0; index < joints.size(); ++index) {
    joints(index) =
      0.5 * (lower_limits[static_cast<std::size_t>(index)] +
      upper_limits[static_cast<std::size_t>(index)]);
  }
  return joints;
}

/**
 * @brief 生成逆运动学求解的初始种子点集合
 * 
 * 为提高逆运动学求解成功率，生成多个不同的初始猜测点。
 * 种子点策略包括：基于目标位置估算首关节角度(yaw)、关节范围中点、腕关节极限位置等。
 * 所有种子点都会经过 clamp_to_limits 确保在关节范围内。
 * 
 * 生成的种子点包含6个：
 * - seed: 原始种子点（钳位后）
 * - yaw_seed: 原始种子点 + 基于目标位置的yaw角度估算
 * - yaw_middle: 关节中点 + yaw角度估算
 * - middle: 所有关节范围中点
 * - lower_wrist: yaw_middle + 腕关节下限
 * - upper_wrist: yaw_middle + 腕关节上限
 * 
 * @param seed 用户提供的初始关节角度猜测
 * @param lower_limits 各关节角度下限（弧度）
 * @param upper_limits 各关节角度上限（弧度）
 * @param target_position 目标末端执行器位置
 * 
 * @return std::vector<JointVector> 包含6个初始种子点的集合
 */
std::vector<JointVector> make_seed_set(
  const JointVector & seed,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & target_position)
{
  const double yaw_guess = std::atan2(target_position.y(), target_position.x());

  JointVector middle = mid_limits(lower_limits, upper_limits);
  JointVector yaw_middle = middle;
  yaw_middle(0) = yaw_guess;

  JointVector yaw_seed = seed;
  yaw_seed(0) = yaw_guess;

  JointVector lower_wrist = yaw_middle;
  lower_wrist(3) = lower_limits[3];

  JointVector upper_wrist = yaw_middle;
  upper_wrist(3) = upper_limits[3];

  return {
    clamp_to_limits(seed, lower_limits, upper_limits),
    clamp_to_limits(yaw_seed, lower_limits, upper_limits),
    clamp_to_limits(yaw_middle, lower_limits, upper_limits),
    clamp_to_limits(middle, lower_limits, upper_limits),
    clamp_to_limits(lower_wrist, lower_limits, upper_limits),
    clamp_to_limits(upper_wrist, lower_limits, upper_limits),
  };
}

Eigen::Matrix<double, 3, 4> numerical_jacobian(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const JointVector & joints)
{
  Eigen::Matrix<double, 3, 4> jacobian;
  constexpr double kEpsilon = 1.0e-5;

  for (int index = 0; index < joints.size(); ++index) {
    JointVector plus = joints;
    JointVector minus = joints;
    plus(index) = std::min(upper_limits[static_cast<std::size_t>(index)], plus(index) + kEpsilon);
    minus(index) = std::max(lower_limits[static_cast<std::size_t>(index)], minus(index) - kEpsilon);

    const double denominator = plus(index) - minus(index);
    if (std::abs(denominator) < 1.0e-12) {
      jacobian.col(index).setZero();
      continue;
    }

    const Eigen::Vector3d position_plus =
      compute_end_effector_position(joint_origins, joint_axes, plus, tool_offset);
    const Eigen::Vector3d position_minus =
      compute_end_effector_position(joint_origins, joint_axes, minus, tool_offset);
    jacobian.col(index) = (position_plus - position_minus) / denominator;
  }

  return jacobian;
}

Eigen::Matrix<double, 6, 1> pose_direction_error(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_tool_x_direction,
  const double direction_weight,
  const JointVector & joints)
{
  const Eigen::Isometry3d transform =
    compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);
  const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();

  Eigen::Matrix<double, 6, 1> error;
  error.segment<3>(0) = target_position - transform.translation();
  error.segment<3>(3) = direction_weight * (target_tool_x_direction - current_tool_x);
  return error;
}

Eigen::Matrix<double, 6, 1> pose_direction_state(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  const double direction_weight,
  const JointVector & joints)
{
  const Eigen::Isometry3d transform =
    compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);
  const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();

  Eigen::Matrix<double, 6, 1> state;
  state.segment<3>(0) = transform.translation();
  state.segment<3>(3) = direction_weight * current_tool_x;
  return state;
}

Eigen::Matrix<double, 6, 4> numerical_pose_direction_jacobian(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const double direction_weight,
  const JointVector & joints)
{
  Eigen::Matrix<double, 6, 4> jacobian;
  constexpr double kEpsilon = 1.0e-5;

  for (int index = 0; index < joints.size(); ++index) {
    JointVector plus = joints;
    JointVector minus = joints;
    plus(index) = std::min(upper_limits[static_cast<std::size_t>(index)], plus(index) + kEpsilon);
    minus(index) = std::max(lower_limits[static_cast<std::size_t>(index)], minus(index) - kEpsilon);

    const double denominator = plus(index) - minus(index);
    if (std::abs(denominator) < 1.0e-12) {
      jacobian.col(index).setZero();
      continue;
    }

    jacobian.col(index) =
      (pose_direction_state(
        joint_origins,
        joint_axes,
        tool_offset,
        direction_weight,
        plus) -
      pose_direction_state(
        joint_origins,
        joint_axes,
        tool_offset,
        direction_weight,
        minus)) /
      denominator;
  }

  return jacobian;
}

bool vector_is_finite(const JointVector & vector)
{
  for (int index = 0; index < vector.size(); ++index) {
    if (!std::isfinite(vector(index))) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool solve_inverse_kinematics(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const JointVector & seed,
  const double tolerance,
  const int max_iterations,
  const double damping,
  const double max_step,
  JointVector & solution,
  double & final_error,
  int & iterations)
{
  const double accepted_tolerance = std::max(1.0e-6, tolerance);
  const double damping_squared = std::max(1.0e-8, damping * damping);
  const double accepted_max_step = std::max(1.0e-4, max_step);
  const int accepted_max_iterations = std::max(1, max_iterations);

  solution = clamp_to_limits(seed, lower_limits, upper_limits);
  final_error = std::numeric_limits<double>::infinity();
  iterations = 0;
  bool found_solution = false;
  double best_solution_score = std::numeric_limits<double>::infinity();
  JointVector best_failed_solution = solution;
  double best_failed_error = final_error;
  int best_failed_iterations = 0;

  auto time_start = std::chrono::steady_clock::now();

  for (const JointVector & initial_joints :
    make_seed_set(seed, lower_limits, upper_limits, target_position))
  {
    JointVector joints = initial_joints;

    for (int iteration = 0; iteration < accepted_max_iterations; ++iteration) {
      if (iteration % 10 == 0) {
        auto elapsed = std::chrono::steady_clock::now() - time_start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 5) {
          break;
        }
      }
      const Eigen::Vector3d current_position =
        compute_end_effector_position(joint_origins, joint_axes, joints, tool_offset);
      const Eigen::Vector3d error = target_position - current_position;
      const double error_norm = error.norm();

      if (error_norm < best_failed_error) {
        best_failed_solution = joints;
        best_failed_error = error_norm;
        best_failed_iterations = iteration + 1;
      }

      if (error_norm <= accepted_tolerance) {
        const double score = error_norm + 0.02 * (joints - seed).norm();
        if (score < best_solution_score) {
          found_solution = true;
          best_solution_score = score;
          solution = joints;
          final_error = error_norm;
          iterations = iteration + 1;
        }
        break;
      }

      const Eigen::Matrix<double, 3, 4> jacobian =
        numerical_jacobian(joint_origins, joint_axes, lower_limits, upper_limits, tool_offset, joints);
      Eigen::Matrix3d damped_system = jacobian * jacobian.transpose();
      damped_system.diagonal().array() += damping_squared;

      JointVector delta =
        jacobian.transpose() * damped_system.ldlt().solve(error);

      if (!vector_is_finite(delta)) {
        break;
      }

      const double delta_norm = delta.norm();
      if (delta_norm > accepted_max_step) {
        delta *= accepted_max_step / delta_norm;
      }

      const JointVector next_joints =
        clamp_to_limits(joints + delta, lower_limits, upper_limits);

      if ((next_joints - joints).norm() < 1.0e-10) {
        break;
      }

      joints = next_joints;
    }
  }

  if (found_solution) {
    return true;
  }

  solution = best_failed_solution;
  final_error = best_failed_error;
  iterations = best_failed_iterations;
  return false;
}

bool solve_inverse_kinematics_with_tool_direction(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_tool_x_direction,
  const JointVector & seed,
  const double position_tolerance,
  const double direction_tolerance,
  const double direction_weight,
  const int max_iterations,
  const double damping,
  const double max_step,
  JointVector & solution,
  double & final_position_error,
  double & final_direction_error,
  int & iterations)
{
  const double accepted_position_tolerance = std::max(1.0e-6, position_tolerance);
  const double accepted_direction_tolerance = std::max(1.0e-6, direction_tolerance);
  const double accepted_direction_weight = std::max(1.0e-6, direction_weight);
  const double damping_squared = std::max(1.0e-8, damping * damping);
  const double accepted_max_step = std::max(1.0e-4, max_step);
  const int accepted_max_iterations = std::max(1, max_iterations);
  const Eigen::Vector3d accepted_target_direction =
    target_tool_x_direction.squaredNorm() > 1.0e-12 ?
    target_tool_x_direction.normalized() :
    Eigen::Vector3d::UnitX();

  solution = clamp_to_limits(seed, lower_limits, upper_limits);
  final_position_error = std::numeric_limits<double>::infinity();
  final_direction_error = std::numeric_limits<double>::infinity();
  iterations = 0;

  bool found_solution = false;
  double best_solution_score = std::numeric_limits<double>::infinity();
  JointVector best_failed_solution = solution;
  double best_failed_score = std::numeric_limits<double>::infinity();
  int best_failed_iterations = 0;

  auto time_start = std::chrono::steady_clock::now();

  for (const JointVector & initial_joints :
    make_seed_set(seed, lower_limits, upper_limits, target_position))
  {
    JointVector joints = initial_joints;

    for (int iteration = 0; iteration < accepted_max_iterations; ++iteration) {
      if (iteration % 10 == 0) {
        auto elapsed = std::chrono::steady_clock::now() - time_start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 5) {
          break;
        }
      }
      const Eigen::Isometry3d transform =
        compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);
      const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();
      const double position_error = (target_position - transform.translation()).norm();
      const double direction_error = (accepted_target_direction - current_tool_x).norm();
      const double combined_score =
        position_error + accepted_direction_weight * direction_error;

      if (combined_score < best_failed_score) {
        best_failed_solution = joints;
        best_failed_score = combined_score;
        final_position_error = position_error;
        final_direction_error = direction_error;
        best_failed_iterations = iteration + 1;
      }

      if (
        position_error <= accepted_position_tolerance &&
        direction_error <= accepted_direction_tolerance)
      {
        const double score =
          combined_score + 0.02 * (joints - seed).norm();
        if (score < best_solution_score) {
          found_solution = true;
          best_solution_score = score;
          solution = joints;
          final_position_error = position_error;
          final_direction_error = direction_error;
          iterations = iteration + 1;
        }
        break;
      }

      const Eigen::Matrix<double, 6, 1> error =
        pose_direction_error(
        joint_origins,
        joint_axes,
        tool_offset,
        target_position,
        accepted_target_direction,
        accepted_direction_weight,
        joints);
      const Eigen::Matrix<double, 6, 4> jacobian =
        numerical_pose_direction_jacobian(
        joint_origins,
        joint_axes,
        lower_limits,
        upper_limits,
        tool_offset,
        accepted_direction_weight,
        joints);
      Eigen::Matrix<double, 6, 6> damped_system = jacobian * jacobian.transpose();
      damped_system.diagonal().array() += damping_squared;

      JointVector delta =
        jacobian.transpose() * damped_system.ldlt().solve(error);

      if (!vector_is_finite(delta)) {
        break;
      }

      const double delta_norm = delta.norm();
      if (delta_norm > accepted_max_step) {
        delta *= accepted_max_step / delta_norm;
      }

      const JointVector next_joints =
        clamp_to_limits(joints + delta, lower_limits, upper_limits);

      if ((next_joints - joints).norm() < 1.0e-10) {
        break;
      }

      joints = next_joints;
    }
  }

  if (found_solution) {
    return true;
  }

  solution = best_failed_solution;
  iterations = best_failed_iterations;
  return false;
}

}  // namespace arm_controller
