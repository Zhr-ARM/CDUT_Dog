#include <array>
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace arm_controller
{

using JointVector = Eigen::Matrix<double, 4, 1>;

/**
 * @brief 计算机器人正运动学，计算末端执行器的位姿变换矩阵
 * 
 * 基于DH参数法的正运动学计算，通过依次累积各关节的平移和旋转变换，
 * 计算从基座坐标系到工具坐标系的完整位姿变换。
 * 
 * 变换顺序：
 * 1. 初始化单位变换矩阵
 * 2. 对每个关节依次执行：平移到关节原点 -> 绕关节轴旋转
 * 3. 添加工具坐标系偏移
 * 
 * @param joint_origins 各关节原点在其前一关节坐标系中的位置数组（4个关节）
 * @param joint_axes 各关节的旋转轴向量数组（若轴向量模长接近0则跳过旋转）
 * @param joint_positions 各关节的角度位置（4维向量，弧度）
 * @param tool_offset 工具坐标系相对于末端关节的平移偏移量
 * 
 * @return Eigen::Isometry3d 从基座到工具坐标系的完整位姿变换矩阵
 */
Eigen::Isometry3d compute_forward_kinematics(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  for (std::size_t index = 0; index < joint_origins.size(); ++index) {
    transform.translate(joint_origins[index]);

    const Eigen::Vector3d axis = joint_axes[index];
    if (axis.squaredNorm() > 1.0e-12) {
      transform.rotate(Eigen::AngleAxisd(joint_positions(static_cast<int>(index)), axis.normalized()));
    }
  }

  transform.translate(tool_offset);
  return transform;
}

/**
 * @brief 计算机器人末端执行器在世界坐标系中的位置
 * 
 * 通过调用正运动学计算函数，获取完整的位姿变换矩阵，
 * 然后提取其中的平移分量作为末端执行器的位置。
 * 
 * @param joint_origins 各关节原点在其前一关节坐标系中的位置数组
 * @param joint_axes 各关节的旋转轴向量数组
 * @param joint_positions 各关节的角度位置（4维向量）
 * @param tool_offset 工具坐标系相对于末端关节的偏移量
 * 
 * @return Eigen::Vector3d 末端执行器在世界坐标系中的位置坐标
 */
Eigen::Vector3d compute_end_effector_position(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset)
{
  return compute_forward_kinematics(
    joint_origins,
    joint_axes,
    joint_positions,
    tool_offset).translation();
}

}  // namespace arm_controller