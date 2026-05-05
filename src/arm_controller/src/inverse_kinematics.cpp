#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_controller/kinematics_types.hpp"

namespace arm_controller
{

namespace
{

// =========================================================================
// Damped Least-Squares update step (shared by both IK solvers).
// =========================================================================

/// Perform one DLS iteration: compute delta_q from Jacobian and error,
/// then clamp, step-limit, and apply limits.  Returns the new joint vector
/// and sets `stalled` to true when the update is negligible.
///
/// @tparam JacobianType  Eigen matrix type (3×4 or 6×4).
/// @tparam ErrorType     Eigen vector type (3×1 or 6×1).
template<typename JacobianType, typename ErrorType>
JointVector dls_update_step(
  const JacobianType & jacobian,
  const ErrorType & error,
  const JointVector & joints,
  const std::array<double, kJointCount> & lower_limits,
  const std::array<double, kJointCount> & upper_limits,
  double damping_squared,
  double max_step,
  bool & stalled)
{
  stalled = false;

  // Damped system:  J·Jᵀ + λ²·I  (force evaluation to get a writable matrix)
  auto damped = (jacobian * jacobian.transpose()).eval();
  damped.diagonal().array() += damping_squared;

  JointVector delta = jacobian.transpose() * damped.ldlt().solve(error);

  if (!vector_is_finite(delta)) {
    stalled = true;
    return joints;
  }

  const double delta_norm = delta.norm();
  if (delta_norm > max_step) {
    delta *= max_step / delta_norm;
  }

  const JointVector next = clamp_to_limits(joints + delta, lower_limits, upper_limits);
  if ((next - joints).norm() < 1.0e-10) {
    stalled = true;
  }
  return next;
}

}  // namespace

// =========================================================================
// Helper implementations (declared in kinematics_types.hpp)
// =========================================================================

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
  // 第 0 关节通常是底座 yaw。用目标点在 XY 平面的方位角估算底座朝向，
  // 可以让迭代从更接近目标方向的位置开始，减少陷入局部极小值的概率。
  const double yaw_guess = std::atan2(target_position.y(), target_position.x());

  // middle：所有关节位于上下限中点，是一个与当前 seed 无关的保守种子。
  JointVector middle = mid_limits(lower_limits, upper_limits);

  // yaw_middle：在中性姿态基础上，只把底座 yaw 转向目标方位。
  JointVector yaw_middle = middle;
  yaw_middle(0) = yaw_guess;

  // yaw_seed：保留调用者给出的其它关节角，只修正底座 yaw。
  // 这通常能保持“靠近当前构型”的优势，同时改善初始朝向。
  JointVector yaw_seed = seed;
  yaw_seed(0) = yaw_guess;

  // 腕关节在不同极限附近可能对应“肘上 / 肘下”或工具翻转等不同构型。
  // 额外尝试第 4 个关节的上下限，可以扩大求解器探索范围。
  JointVector lower_wrist = yaw_middle;
  lower_wrist(3) = lower_limits[3];

  JointVector upper_wrist = yaw_middle;
  upper_wrist(3) = upper_limits[3];

  return {
    // 所有种子最终都钳位到关节限制内，避免一开始就从非法构型求解。
    clamp_to_limits(seed, lower_limits, upper_limits),
    clamp_to_limits(yaw_seed, lower_limits, upper_limits),
    clamp_to_limits(yaw_middle, lower_limits, upper_limits),
    clamp_to_limits(middle, lower_limits, upper_limits),
    clamp_to_limits(lower_wrist, lower_limits, upper_limits),
    clamp_to_limits(upper_wrist, lower_limits, upper_limits),
  };
}

/**
 * @brief 计算机器人末端位置的数值雅可比矩阵
 *
 * 使用中心差分法（Finite Difference）估算雅可比矩阵。对于每一个关节，
 * 在其当前角度基础上增加和减去一个微小增量 (kEpsilon)，通过计算这两点处
 * 正运动学的位置变化值来近似求取偏导数。
 * 会结合上下限确保差分扰动不超出关节物理边界；若关节贴近边界，则自动退化为单侧差分。
 *
 * 雅可比矩阵第 j 列含义：
 *   J.col(j) = d(position) / d(q_j)
 * 即第 j 个关节角变化 1 rad 时，末端位置在 x/y/z 方向上的局部变化率。
 *
 * @param joint_origins 各关节原点在其前一关节坐标系中的位置
 * @param joint_axes 各关节的旋转轴向量
 * @param lower_limits 各关节的角度下限
 * @param upper_limits 各关节的角度上限
 * @param tool_offset 工具坐标系偏移
 * @param joints 当前计算雅可比矩阵对应的关节角度向量
 *
 * @return Eigen::Matrix<double, 3, 4> 3x4的位置雅可比矩阵（3维空间位置，4个关节变量）
 */
Eigen::Matrix<double, 3, 4> numerical_jacobian(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const JointVector & joints)
{
  // 3 行对应末端位置 x/y/z，4 列对应 4 个关节变量 q0...q3。
  Eigen::Matrix<double, 3, 4> jacobian;

  // 差分步长：足够小以近似导数，同时避免浮点舍入误差过度放大。
  constexpr double kEpsilon = 1.0e-5;

  for (int index = 0; index < joints.size(); ++index) {
    // 只扰动当前关节，其它关节保持不变，从而得到当前关节对应的偏导数列。
    JointVector plus = joints;
    JointVector minus = joints;

    // 差分点必须仍在关节限制内。若当前关节靠近上/下限，plus 或 minus 会被截断。
    plus(index) = std::min(upper_limits[static_cast<std::size_t>(index)], plus(index) + kEpsilon);
    minus(index) = std::max(lower_limits[static_cast<std::size_t>(index)], minus(index) - kEpsilon);

    // 实际差分间隔可能小于 2*kEpsilon，甚至在上下限重合时为 0。
    const double denominator = plus(index) - minus(index);
    if (std::abs(denominator) < 1.0e-12) {
      // 该关节没有可用运动范围，认为它对末端位置没有可求导的贡献。
      jacobian.col(index).setZero();
      continue;
    }

    // 分别计算 q_i + epsilon 和 q_i - epsilon 处的末端位置。
    const Eigen::Vector3d position_plus =
      compute_end_effector_position(joint_origins, joint_axes, plus, tool_offset);
    const Eigen::Vector3d position_minus =
      compute_end_effector_position(joint_origins, joint_axes, minus, tool_offset);

    // 有限差分近似导数：(f(q+h)-f(q-h))/(2h)，边界处使用实际 denominator。
    jacobian.col(index) = (position_plus - position_minus) / denominator;
  }

  return jacobian;
}

/**
 * @brief 计算包含姿态定向的末端运动误差
 *
 * 评估当前机器人末端执行器的实际位置与工具X轴方向，与期望目标之间的差值。
 * 包含 3维位置误差 和 3维方向误差。方向误差通过 direction_weight 进行加权。
 *
 * @param joint_origins 各关节原点
 * @param joint_axes 各关节旋转轴
 * @param tool_offset 工具坐标系偏移
 * @param target_position 期望到达的目标位置
 * @param target_tool_x_direction 期望的工具X轴指向（通常指代工具的“前”向）
 * @param direction_weight 姿态方向误差在总误差中的权重（调节位置与姿态之间的优化偏好）
 * @param joints 当前的关节角度状态
 *
 * @return Eigen::Matrix<double, 6, 1> 6维误差向量（前3个为位置误差，后3个为加权的方向误差）
 */
Eigen::Matrix<double, 6, 1> pose_direction_error(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_tool_x_direction,
  const double direction_weight,
  const JointVector & joints)
{
  // 完整正运动学可同时拿到末端平移 transform.translation()
  // 和末端坐标系朝向 transform.linear()。
  const Eigen::Isometry3d transform =
    compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);

  // Eigen::Isometry3d 的 linear() 是 3x3 旋转矩阵。
  // 第 0 列表示工具坐标系 X 轴在世界坐标系中的方向。
  const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();

  Eigen::Matrix<double, 6, 1> error;

  // 前 3 维：位置误差。方向为“目标 - 当前”，后续 DLS 求解会沿着减小误差的方向更新关节。
  error.segment<3>(0) = target_position - transform.translation();

  // 后 3 维：工具 X 轴方向误差。
  // 注意这里使用两个单位方向向量的差，而不是旋转角误差；其范数范围大约为 [0, 2]。
  // direction_weight 把无量纲方向误差缩放到与位置误差相近的优化量级。
  error.segment<3>(3) = direction_weight * (target_tool_x_direction - current_tool_x);
  return error;
}

/**
 * @brief 获取包含平移与姿态矢量的末端状态向量
 *
 * 计算当前关节状态下，末端执行器的世界坐标系位置以及受权重调节后的工具X轴方向向量。
 * 主要用于在数值雅可比求解过程中，构成代表机器人系统状态的复合向量。
 *
 * @param joint_origins 各关节原点
 * @param joint_axes 各关节旋转轴
 * @param tool_offset 工具坐标系偏移
 * @param direction_weight 姿态方向在状态向量中的权重
 * @param joints 当前的关节角度状态
 *
 * @return Eigen::Matrix<double, 6, 1> 6维状态向量（前3维分量为位置，后3维分量为加权X轴方向）
 */
Eigen::Matrix<double, 6, 1> pose_direction_state(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const Eigen::Vector3d & tool_offset,
  const double direction_weight,
  const JointVector & joints)
{
  // 该函数返回“状态”而不是“误差”，用于对复合状态做有限差分。
  // 状态 = [末端位置; 加权后的工具 X 轴方向]。
  const Eigen::Isometry3d transform =
    compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);
  const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();

  Eigen::Matrix<double, 6, 1> state;

  // 前 3 维直接使用末端世界坐标位置。
  state.segment<3>(0) = transform.translation();

  // 后 3 维使用加权方向，这样对该状态求导得到的雅可比也自然带有同样权重。
  state.segment<3>(3) = direction_weight * current_tool_x;
  return state;
}

/**
 * @brief 计算包含位置与定向姿态的扩展数值雅可比矩阵
 *
 * 使用有限差分法，分别对每个关节添加微小扰动，计算复合状态（位置 + 加权X方向）的阶跃变化率。
 * 从而获得用于阻尼最小二乘法进行六自由度（含权）求解系统的增广雅可比矩阵。
 *
 * 雅可比矩阵第 j 列含义：
 *   J.col(j) = d([position; weight * tool_x]) / d(q_j)
 * 前 3 行描述位置对关节的敏感度，后 3 行描述工具 X 轴方向对关节的敏感度。
 *
 * @param joint_origins 各关节原点
 * @param joint_axes 各关节旋转轴
 * @param lower_limits 关节角度下限
 * @param upper_limits 关节角度上限
 * @param tool_offset 工具偏移
 * @param direction_weight 方向权重，用于统一位置与方向维度的量级差异
 * @param joints 当前关节求导工作点
 *
 * @return Eigen::Matrix<double, 6, 4> 6x4增广雅可比矩阵
 */
Eigen::Matrix<double, 6, 4> numerical_pose_direction_jacobian(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
  const Eigen::Vector3d & tool_offset,
  const double direction_weight,
  const JointVector & joints)
{
  // 6 行对应复合任务空间误差：3 维位置 + 3 维工具方向；4 列对应 4 个关节。
  Eigen::Matrix<double, 6, 4> jacobian;
  constexpr double kEpsilon = 1.0e-5;

  for (int index = 0; index < joints.size(); ++index) {
    // 与纯位置雅可比一致：每次只扰动一个关节，计算复合状态变化率。
    JointVector plus = joints;
    JointVector minus = joints;
    plus(index) = std::min(upper_limits[static_cast<std::size_t>(index)], plus(index) + kEpsilon);
    minus(index) = std::max(lower_limits[static_cast<std::size_t>(index)], minus(index) - kEpsilon);

    const double denominator = plus(index) - minus(index);
    if (std::abs(denominator) < 1.0e-12) {
      // 无可用差分间隔时，该列置零，避免除以 0 产生 NaN。
      jacobian.col(index).setZero();
      continue;
    }

    // 这里对 pose_direction_state 求导，而不是对误差求导。
    // 由于误差 = target_state - current_state，DLS 更新中使用 J^T * error，
    // 该符号约定会推动 current_state 朝 target_state 靠近。
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

// =========================================================================
// Position-only IK solver
// =========================================================================

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
  // 对外部传入的参数做防御性下限保护，避免 0 或负数参数导致求解器不可用。
  // 这些 accepted_* 只影响本次求解，不会修改调用者传入的原始参数。
  const double accepted_tolerance = std::max(1.0e-6, tolerance);
  const double damping_squared = std::max(1.0e-8, damping * damping);
  const double accepted_max_step = std::max(1.0e-4, max_step);
  const int accepted_max_iterations = std::max(1, max_iterations);

  // 默认输出先设为钳位后的 seed；如果后续完全没有更好结果，也能返回一个合法关节构型。
  solution = clamp_to_limits(seed, lower_limits, upper_limits);
  final_error = std::numeric_limits<double>::infinity();
  iterations = 0;

  // found_solution 表示是否至少有一个种子收敛到 tolerance 以内。
  bool found_solution = false;

  // 多个种子都成功时，用 score 选择一个更优解：
  // 误差越小越好，同时轻微偏好离 seed 更近的构型，减少不必要的大幅运动。
  double best_solution_score = std::numeric_limits<double>::infinity();

  // 如果全部种子都失败，仍然记录误差最小的中间构型，作为 best-effort 返回。
  JointVector best_failed_solution = solution;
  double best_failed_error = final_error;
  int best_failed_iterations = 0;

  // 计时覆盖全部种子，而不是每个种子单独计时。
  auto time_start = std::chrono::steady_clock::now();

  for (const JointVector & initial_joints :
    make_seed_set(seed, lower_limits, upper_limits, target_position))
  {
    // 每个种子独立迭代，不把一个种子的失败结果继续传给下一个种子。
    JointVector joints = initial_joints;

    for (int iteration = 0; iteration < accepted_max_iterations; ++iteration) {
      // 不是每一轮都读取时钟，减少 steady_clock 调用对控制循环的额外开销。
      if (iteration % 10 == 0) {
        auto elapsed = std::chrono::steady_clock::now() - time_start;
        if (elapsed > kSolverTimeBudget) {
          // 当前种子提前停止；外层会继续检查下一个种子，但因为共享总时间预算，
          // 后续种子也会很快退出。
          break;
        }
      }

      // 当前正运动学位置。
      const Eigen::Vector3d current_position =
        compute_end_effector_position(joint_origins, joint_axes, joints, tool_offset);

      // 任务空间误差：希望末端从 current_position 移动到 target_position。
      const Eigen::Vector3d error = target_position - current_position;
      const double error_norm = error.norm();

      // 随时保存失败路径上的最佳点，保证求解失败时输出尽可能接近目标的位置。
      if (error_norm < best_failed_error) {
        best_failed_solution = joints;
        best_failed_error = error_norm;
        best_failed_iterations = iteration + 1;
      }

      if (error_norm <= accepted_tolerance) {
        // 达到位置容差即认为当前种子成功。
        // score 中加入 (joints - seed).norm()，使解更接近调用者的初始构型。
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

      // Numerical Jacobian (3×4) and DLS update.
      const Eigen::Matrix<double, 3, 4> jacobian =
        numerical_jacobian(joint_origins, joint_axes, lower_limits, upper_limits, tool_offset, joints);

      bool stalled = false;
      joints = dls_update_step(jacobian, error, joints,
                               lower_limits, upper_limits,
                               damping_squared, accepted_max_step, stalled);
      if (stalled) {
        break;
      }
    }
  }

  if (found_solution) {
    // 至少一个种子成功时，solution / final_error / iterations 已经保存了最佳成功解。
    return true;
  }

  // 全部种子失败时返回搜索过程中距离目标最近的构型，方便上层决定是否执行或报警。
  solution = best_failed_solution;
  final_error = best_failed_error;
  iterations = best_failed_iterations;
  return false;
}

/**
 * @brief 求解匹配位置并限制工具指向的增强逆运动学
 *
 * 相比基础的 solve_inverse_kinematics，本函数在迭代中不仅最小化位置误差，
 * 同时还尝试使工具末端坐标系的 X 轴与用户指定的目标方向向量对齐。
 * 目标函数综合了位置及方向误差的加权和进行求解探索。
 *
 * @param joint_origins 各关节原点
 * @param joint_axes 各关节旋转轴
 * @param lower_limits 关节角度下限
 * @param upper_limits 关节角度上限
 * @param tool_offset 工具偏移
 * @param target_position 期望的 3D 目标坐标
 * @param target_tool_x_direction 期望的工具 X 轴空间指向向量
 * @param seed 初始猜测构型
 * @param position_tolerance 可接受的位置绝对误差阈值
 * @param direction_tolerance 可接受的方向向量欧式距离误差阈值
 * @param direction_weight 在组合损失函数中，针对方向误差维度的相对权重，调大以优先满足姿态
 * @param max_iterations 最大迭代次数
 * @param damping DLS 阻尼因子
 * @param max_step 迭代步长限制
 * @param solution [输出] 求解成功的关节构型 或 失败时的次优构型
 * @param final_position_error [输出] 最终的位置残差
 * @param final_direction_error [输出] 最终的方向残差
 * @param iterations [输出] 实际消耗的迭代总计
 *
 * @return true 找到位置与姿态误差同时在容差范围内的有效解
 * @return false 未能找到满足双重容差的解
 */
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
  // 参数防御性处理：容差、权重、阻尼、步长、迭代次数都必须有合理正下限。
  const double accepted_position_tolerance = std::max(1.0e-6, position_tolerance);
  const double accepted_direction_tolerance = std::max(1.0e-6, direction_tolerance);
  const double accepted_direction_weight = std::max(1.0e-6, direction_weight);
  const double damping_squared = std::max(1.0e-8, damping * damping);
  const double accepted_max_step = std::max(1.0e-4, max_step);
  const int accepted_max_iterations = std::max(1, max_iterations);

  // 方向目标必须是单位向量；若调用者传入零向量，就退回到工具 X 轴正方向。
  // 这样可以避免 normalized() 对零向量产生未定义数值。
  const Eigen::Vector3d accepted_target_direction =
    target_tool_x_direction.squaredNorm() > 1.0e-12 ?
    target_tool_x_direction.normalized() :
    Eigen::Vector3d::UnitX();

  // 输出初始值保持为合法构型；后续成功或失败都会覆盖为更有意义的结果。
  solution = clamp_to_limits(seed, lower_limits, upper_limits);
  final_position_error = std::numeric_limits<double>::infinity();
  final_direction_error = std::numeric_limits<double>::infinity();
  iterations = 0;

  // 与纯位置 IK 类似：成功解按 score 选最佳，失败解按 combined_score 选最接近目标的构型。
  bool found_solution = false;
  double best_solution_score = std::numeric_limits<double>::infinity();
  JointVector best_failed_solution = solution;
  double best_failed_score = std::numeric_limits<double>::infinity();
  int best_failed_iterations = 0;

  auto time_start = std::chrono::steady_clock::now();

  for (const JointVector & initial_joints :
    make_seed_set(seed, lower_limits, upper_limits, target_position))
  {
    // 每个初始种子独立运行一轮 DLS。
    JointVector joints = initial_joints;

    for (int iteration = 0; iteration < accepted_max_iterations; ++iteration) {
      // 全局时间预算保护，避免增强 IK 因为 6 维任务计算更重而占用控制线程太久。
      if (iteration % 10 == 0) {
        auto elapsed = std::chrono::steady_clock::now() - time_start;
        if (elapsed > kSolverTimeBudget) {
          break;
        }
      }
      const Eigen::Isometry3d transform =
        compute_forward_kinematics(joint_origins, joint_axes, joints, tool_offset);

      // 当前工具 X 轴方向，用于衡量末端“朝向”是否满足要求。
      const Eigen::Vector3d current_tool_x = transform.linear().col(0).normalized();

      // 分别计算位置残差和方向残差，便于用两个独立容差判断成功。
      const double position_error = (target_position - transform.translation()).norm();
      const double direction_error = (accepted_target_direction - current_tool_x).norm();

      // combined_score 用于比较失败路径和成功解优劣。
      // direction_weight 越大，求解器越倾向于牺牲一点位置误差来改善工具方向。
      const double combined_score =
        position_error + accepted_direction_weight * direction_error;

      // 记录失败情况下的最佳点。
      // final_position_error / final_direction_error 同步更新，保证 false 返回时也有有效诊断信息。
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
        // 只有位置和方向同时达标才认为增强 IK 成功。
        // 与纯位置 IK 一样，轻微偏好离 seed 更近的解。
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

      // 构造 6 维误差向量：[位置误差; 加权方向误差]。
      const Eigen::Matrix<double, 6, 1> error =
        pose_direction_error(
        joint_origins,
        joint_axes,
        tool_offset,
        target_position,
        accepted_target_direction,
        accepted_direction_weight,
        joints);

      // 构造 6x4 增广雅可比，把关节增量映射到“位置 + 工具 X 轴方向”的变化。
      const Eigen::Matrix<double, 6, 4> jacobian =
        numerical_pose_direction_jacobian(
        joint_origins,
        joint_axes,
        lower_limits,
        upper_limits,
        tool_offset,
        accepted_direction_weight,
        joints);

      bool stalled = false;
      joints = dls_update_step(jacobian, error, joints,
                               lower_limits, upper_limits,
                               damping_squared, accepted_max_step, stalled);
      if (stalled) {
        break;
      }
    }
  }

  if (found_solution) {
    // 返回最佳成功解。
    return true;
  }

  // 返回最佳失败解；final_position_error / final_direction_error 已在记录最佳点时更新。
  solution = best_failed_solution;
  iterations = best_failed_iterations;
  return false;
}

}  // namespace arm_controller
