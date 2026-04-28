#ifndef DOG_POSITION_CONTROL_ROBOT_MODEL_HPP_
#define DOG_POSITION_CONTROL_ROBOT_MODEL_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace dog_position_control
{

// 整机固定拓扑:
// - 4 条腿
// - 每条腿 3 个关节: HAA / HFE / KFE
// - 每个关节的输出命令都采用 5 个槽位的 MIT 风格协议
inline constexpr size_t kLegCount = 4;
inline constexpr size_t kJointsPerLeg = 3;
inline constexpr size_t kValuesPerJoint = 5;
inline constexpr size_t kLegCommandSize = kJointsPerLeg * kValuesPerJoint;
inline constexpr size_t kAggregateCommandSize = kLegCount * kLegCommandSize;
inline constexpr size_t kTotalJointCount = kLegCount * kJointsPerLeg;
inline constexpr size_t kHaaJointIndex = 0;
inline constexpr size_t kHfeJointIndex = 1;
inline constexpr size_t kKfeJointIndex = 2;

// CDUT 模型的关节限位，需要与 dog_bringup/xacro/cdut_dog/const.xacro 保持一致。
inline constexpr double kHaaLower = -0.5;
inline constexpr double kHaaUpper = 0.5;
inline constexpr double kHfeLower = -2.0;
inline constexpr double kHfeUpper = 1.6;
inline constexpr double kKfeLower = -2.0;
inline constexpr double kKfeUpper = 2.0;

inline const std::array<std::string, kLegCount> kLegNames = {"LF", "LH", "RF", "RH"};
inline const std::array<std::string, kJointsPerLeg> kJointSuffixes = {"HAA", "HFE", "KFE"};

// 把二维索引 [腿, 关节] 映射到长度为 12 的扁平关节数组中。
constexpr size_t flat_joint_index(size_t leg_index, size_t joint_index)
{
  return leg_index * kJointsPerLeg + joint_index;
}

// 为一条腿缓存三个关节在扁平数组中的索引，避免后续频繁手写偏移。
struct JointIndices
{
  size_t haa;
  size_t hfe;
  size_t kfe;
};

constexpr JointIndices joint_indices(size_t leg_index)
{
  return {
    flat_joint_index(leg_index, kHaaJointIndex),
    flat_joint_index(leg_index, kHfeJointIndex),
    flat_joint_index(leg_index, kKfeJointIndex)};
}

inline std::string joint_name(size_t leg_index, size_t joint_index)
{
  return kLegNames[leg_index] + "_" + kJointSuffixes[joint_index];
}

// 当前命名约定下，LF/RF 视为前腿，LH/RH 视为后腿。
inline bool is_front_leg(size_t leg_index)
{
  return leg_index == 0 || leg_index == 2;
}

// 腿部几何参数。这里采用平面二连杆近似，只关心 HFE-KFE 平面。
struct LegGeometry
{
  double thigh_length{0.2};
  double shank_length{0.22};
};

// 逆运动学输出的两个俯仰关节目标角。
struct IKResult
{
  double hfe{0.0};
  double kfe{0.0};
};

// 足端轨迹在腿平面内的位姿和速度。
// x: 前后方向
// y: 向下的支撑深度方向
struct FootState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
};

// 由足端速度映射得到的关节角速度。
struct JointVel
{
  double hfe{0.0};
  double kfe{0.0};
};

// 一条腿三个关节的目标角集合。
struct JointTargets
{
  double haa{0.0};
  double hfe{0.0};
  double kfe{0.0};
};

// 正运动学返回的简化足端位置。
struct LegPose2D
{
  double x{0.0};
  double y{0.0};
};

// 简单一阶低通滤波器。
class LowPassFilter
{
public:
  LowPassFilter() = default;

  explicit LowPassFilter(double alpha)
  : alpha_(alpha)
  {
  }

  void set_alpha(double alpha)
  {
    alpha_ = std::clamp(alpha, 0.01, 1.0);
  }

  double filter(double input)
  {
    if (!initialized_)
    {
      value_ = input;
      initialized_ = true;
    }
    else
    {
      value_ = alpha_ * input + (1.0 - alpha_) * value_;
    }
    return value_;
  }

private:
  double alpha_{0.3};
  bool initialized_{false};
  double value_{0.0};
};

// 每条腿在控制循环中需要维护的状态。
struct LegState
{
  LowPassFilter hfe_filter;
  LowPassFilter kfe_filter;
  LowPassFilter hfe_velocity_filter;
  LowPassFilter kfe_velocity_filter;
  double hfe_offset{0.0};
  double kfe_offset{0.0};
  double thigh_mass{0.0};
  double shank_mass{0.0};
};

inline double signed_target(const std::vector<double> & signs, size_t leg_index, double value)
{
  if (leg_index >= signs.size())
  {
    return value;
  }
  return signs[leg_index] * value;
}

inline double direct_target_value(
  const std::vector<double> & values,
  const std::vector<double> & signs,
  size_t leg_index,
  double lower,
  double upper)
{
  if (leg_index >= values.size())
  {
    return 0.0;
  }
  return std::clamp(signed_target(signs, leg_index, values[leg_index]), lower, upper);
}

}  // namespace dog_position_control

#endif  // DOG_POSITION_CONTROL_ROBOT_MODEL_HPP_
