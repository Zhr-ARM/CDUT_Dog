#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "dog_position_control/command_layout.hpp"
#include "dog_position_control/gait_schedule.hpp"
#include "dog_position_control/gait_trajectory.hpp"
#include "dog_position_control/leg_kinematics.hpp"
#include "dog_position_control/math_utils.hpp"
#include "dog_position_control/robot_model.hpp"

// 这个节点是整机级的高层步态控制器。
//
// 它的职责不是直接驱动单个执行器，而是:
// 1. 结合步态参数生成四条腿的足端轨迹。
// 2. 通过 2D 腿部逆运动学把足端目标转换为关节目标。
// 3. 叠加启动站立状态机、平滑切换、增益缩放和重力前馈。
// 4. 最终按 MIT 风格五元组 [p, v, t_ff, kp, kd] 组织为扁平数组，
//    发布到 /motor_cmd，供 mit_controller 或硬件节点消费。
//
// 代码结构上可以把本文件分成三层:
// - dog_position_control 公共模块: 数学工具、运动学、轨迹函数、命令布局
// - QuadrupedGaitController: 参数加载、状态机、主控制循环
// - main(): 普通 ROS 2 节点入口

namespace dog_position_control
{
namespace
{

std::string to_lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

// 所有按腿配置的参数都必须严格是 4 个元素，对应 LF/LH/RF/RH。
void validate_vector_size(const std::string & name, const std::vector<double> & values)
{
  if (values.size() != kLegCount)
  {
    throw std::runtime_error(name + " must contain exactly 4 values.");
  }
}

struct NamedVectorParameter
{
  const char * name;
  const std::vector<double> * values;
};

// 批量校验所有“每条腿一组”的参数，防止某个参数长度错误后在控制循环里
// 才触发越界访问。
void validate_vector_sizes(std::initializer_list<NamedVectorParameter> parameters)
{
  for (const auto & parameter : parameters)
  {
    validate_vector_size(parameter.name, *parameter.values);
  }
}

}  // namespace

// 整机步态控制节点。
//
// 输入:
// - /joint_states: 当前关节位置，用于站立初值捕获、支撑高度估计和偏置修正
//
// 输出:
// - /motor_cmd: 面向 MIT 风格控制器/驱动的扁平化五元组命令
//
// 核心控制链路:
// 1. 根据 gait_profile 和 phase_offsets 生成四条腿的局部相位
// 2. 计算摆动/支撑足端轨迹
// 3. 通过 IK 与速度映射得到关节目标和目标速度
// 4. 叠加站立状态机、命令平滑、增益缩放、重力前馈
// 5. 输出每个关节的 [p, v, t_ff, kp, kd]
class QuadrupedGaitController : public rclcpp::Node
{
public:
  // 启动阶段不是直接从“关机姿态”跳到“步态行走”，而是经过三个明确阶段:
  // - kStandUp: 从当前实际关节位置平滑抬到站姿
  // - kStandHold: 先站稳一小段时间
  // - kWalk:    再逐步混入步态轨迹
  enum class MotionState
  {
    kStandUp,
    kStandHold,
    kWalk,
  };

  enum class AutoSequenceStage
  {
    kInactive,
    kInitialStand,
    kInPlaceStep,
    kForward,
    kBackward,
    kTurnRight,
    kTurnLeft,
    kFinalStand,
  };

  QuadrupedGaitController()
  : Node("quadruped_gait_controller")
  {
    initialize_joint_name_to_index();

    // 第一组参数描述的是整机运动学/步态节奏/启动策略/前馈与增益等
    // “标量配置”，它们决定控制器的基本行为。
    command_topic_ = declare_parameter<std::string>("command_topic", "/motor_cmd");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    gait_action_topic_ = declare_parameter<std::string>("gait_action_topic", "/gait_action");
    gait_profile_ = to_lower_copy(declare_parameter<std::string>("gait_profile", "legacy_walk"));
    step_length_ = declare_parameter<double>("step_length", 0.10);
    step_height_ = declare_parameter<double>("step_height", 0.05);
    gait_period_ = declare_parameter<double>("gait_period", 3.0);
    stance_depth_ = declare_parameter<double>("stance_depth", 0.33);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    swing_ratio_ = declare_parameter<double>("swing_ratio", 0.7);
    stance_transition_ratio_ = declare_parameter<double>("stance_transition_ratio", 0.15);
    reset_phase_on_walk_start_ = declare_parameter<bool>("reset_phase_on_walk_start", true);
    walk_start_phase_ = declare_parameter<double>("walk_start_phase", 0.0);
    swing_gain_scale_ = declare_parameter<double>("swing_gain_scale", 0.5);
    swing_hfe_motion_scale_ = declare_parameter<double>("swing_hfe_motion_scale", 0.45);
    swing_extra_kfe_flexion_ = declare_parameter<double>("swing_extra_kfe_flexion", 0.18);
    use_direct_joint_in_place_step_ =
      declare_parameter<bool>("use_direct_joint_in_place_step", false);
    direct_step_hfe_amplitude_ =
      declare_parameter<double>("direct_step_hfe_amplitude", 0.0);
    direct_step_kfe_amplitude_ =
      declare_parameter<double>("direct_step_kfe_amplitude", 0.0);
    direct_step_support_hfe_amplitude_ =
      declare_parameter<double>("direct_step_support_hfe_amplitude", 0.0);
    direct_step_support_kfe_amplitude_ =
      declare_parameter<double>("direct_step_support_kfe_amplitude", 0.0);
    direct_step_hfe_signs_ =
      declare_parameter<std::vector<double>>("direct_step_hfe_signs", {1.0, 1.0, 1.0, 1.0});
    direct_step_kfe_signs_ =
      declare_parameter<std::vector<double>>("direct_step_kfe_signs", {1.0, 1.0, 1.0, 1.0});
    direct_step_hfe_swing_scales_ =
      declare_parameter<std::vector<double>>(
        "direct_step_hfe_swing_scales", {1.0, 1.0, 1.0, 1.0});
    direct_step_kfe_swing_scales_ =
      declare_parameter<std::vector<double>>(
        "direct_step_kfe_swing_scales", {1.0, 1.0, 1.0, 1.0});
    direct_step_hfe_support_scales_ =
      declare_parameter<std::vector<double>>(
        "direct_step_hfe_support_scales", {1.0, 1.0, 1.0, 1.0});
    direct_step_kfe_support_scales_ =
      declare_parameter<std::vector<double>>(
        "direct_step_kfe_support_scales", {1.0, 1.0, 1.0, 1.0});
    rear_swing_step_length_scale_ = declare_parameter<double>("rear_swing_step_length_scale", 1.0);
    rear_swing_step_height_scale_ = declare_parameter<double>("rear_swing_step_height_scale", 1.0);
    rear_swing_gain_scale_ = declare_parameter<double>("rear_swing_gain_scale", 1.0);
    rear_swing_hfe_motion_scale_ = declare_parameter<double>("rear_swing_hfe_motion_scale", 1.0);
    rear_swing_extra_kfe_flexion_scale_ =
      declare_parameter<double>("rear_swing_extra_kfe_flexion_scale", 1.0);
    rear_swing_pitch_feedforward_offset_ =
      declare_parameter<double>("rear_swing_pitch_feedforward_offset", 0.0);
    rear_swing_body_x_shift_ = declare_parameter<double>("rear_swing_body_x_shift", 0.0);
    support_pitch_balance_gain_ = declare_parameter<double>("support_pitch_balance_gain", 0.0);
    support_pitch_balance_max_offset_ =
      declare_parameter<double>("support_pitch_balance_max_offset", 0.0);
    support_pitch_balance_target_delta_ =
      declare_parameter<double>("support_pitch_balance_target_delta", 0.0);
    const double filter_alpha = declare_parameter<double>("filter_alpha", 0.4);
    stand_only_ = declare_parameter<bool>("stand_only", true);
    enable_startup_stand_ = declare_parameter<bool>("enable_startup_stand", false);
    use_direct_joint_stand_targets_ =
      declare_parameter<bool>("use_direct_joint_stand_targets", true);
    use_startup_crouch_pose_ = declare_parameter<bool>("use_startup_crouch_pose", false);
    allow_partial_joint_state_startup_ =
      declare_parameter<bool>("allow_partial_joint_state_startup", false);
    joint_isolation_mode_ = declare_parameter<std::string>("joint_isolation_mode", "disabled");
    joint_isolation_kfe_target_ = declare_parameter<double>("joint_isolation_kfe_target", -0.70);
    startup_settle_duration_ = declare_parameter<double>("startup_settle_duration", 2.0);
    startup_min_support_scale_ = declare_parameter<double>("startup_min_support_scale", 0.05);
    startup_initial_gain_scale_ = declare_parameter<double>("startup_initial_gain_scale", 0.15);
    stand_up_duration_ = declare_parameter<double>("stand_up_duration", 3.0);
    startup_knee_first_ratio_ = declare_parameter<double>("startup_knee_first_ratio", 0.60);
    startup_front_legs_first_ = declare_parameter<bool>("startup_front_legs_first", false);
    startup_front_leg_complete_ratio_ =
      declare_parameter<double>("startup_front_leg_complete_ratio", 1.0);
    startup_rear_hip_start_ratio_ =
      declare_parameter<double>("startup_rear_hip_start_ratio", 0.0);
    startup_rear_knee_start_ratio_ =
      declare_parameter<double>("startup_rear_knee_start_ratio", 0.0);
    stand_hold_duration_ = declare_parameter<double>("stand_hold_duration", 1.0);
    walk_ramp_duration_ = declare_parameter<double>("walk_ramp_duration", 1.0);
    stand_gain_scale_ = declare_parameter<double>("stand_gain_scale", 1.8);
    walk_gain_scale_ = declare_parameter<double>("walk_gain_scale", 1.0);
    enable_auto_sequence_ = declare_parameter<bool>("enable_auto_sequence", false);
    auto_sequence_initial_stand_duration_ =
      declare_parameter<double>("auto_sequence_initial_stand_duration", 2.0);
    auto_sequence_in_place_step_duration_ =
      declare_parameter<double>("auto_sequence_in_place_step_duration", 4.0);
    auto_sequence_forward_duration_ =
      declare_parameter<double>("auto_sequence_forward_duration", 5.0);
    auto_sequence_backward_duration_ =
      declare_parameter<double>("auto_sequence_backward_duration", 5.0);
    auto_sequence_turn_right_duration_ =
      declare_parameter<double>("auto_sequence_turn_right_duration", 5.0);
    auto_sequence_turn_left_duration_ =
      declare_parameter<double>("auto_sequence_turn_left_duration", 5.0);
    auto_sequence_forward_use_direct_step_ =
      declare_parameter<bool>("auto_sequence_forward_use_direct_step", true);
    auto_sequence_forward_step_length_ =
      declare_parameter<double>("auto_sequence_forward_step_length", 0.02);
    auto_sequence_backward_step_length_ =
      declare_parameter<double>("auto_sequence_backward_step_length", -0.02);
    auto_sequence_turn_step_length_ =
      declare_parameter<double>("auto_sequence_turn_step_length", 0.015);
    auto_sequence_turn_left_step_length_ =
      declare_parameter<double>("auto_sequence_turn_left_step_length", -0.015);
    auto_sequence_forward_step_height_ =
      declare_parameter<double>("auto_sequence_forward_step_height", 0.012);
    auto_sequence_forward_hfe_amplitude_ =
      declare_parameter<double>("auto_sequence_forward_hfe_amplitude", 0.02);
    auto_sequence_backward_hfe_amplitude_ =
      declare_parameter<double>("auto_sequence_backward_hfe_amplitude", 0.02);
    auto_sequence_turn_hfe_amplitude_ =
      declare_parameter<double>("auto_sequence_turn_hfe_amplitude", 0.018);
    stand_haa_slew_rate_ = declare_parameter<double>("stand_haa_slew_rate", 0.15);
    stand_hfe_slew_rate_ = declare_parameter<double>("stand_hfe_slew_rate", 0.20);
    stand_kfe_slew_rate_ = declare_parameter<double>("stand_kfe_slew_rate", 0.25);
    gravity_ff_scale_ = declare_parameter<double>("gravity_ff_scale", 2.2);
    shank_mass_ratio_ = declare_parameter<double>("shank_mass_ratio", 0.5);
    debug_joint_tracking_ = declare_parameter<bool>("debug_joint_tracking", false);
    debug_log_period_ = declare_parameter<double>("debug_log_period", 0.5);

    // cmd_vel 遥控参数
    const std::string cmd_vel_topic =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.5);
    max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.0);
    cmd_vel_deadzone_ = declare_parameter<double>("cmd_vel_deadzone", 0.05);

    // 第二组参数是“按腿配置”的数组，约定顺序统一为 LF/LH/RF/RH。
    // 这类参数允许对前后腿或左右腿做不对称补偿。
    phase_offsets_ =
      declare_parameter<std::vector<double>>("phase_offsets", {0.0, 0.5, 0.5, 0.0});
    foot_x_signs_ =
      declare_parameter<std::vector<double>>("foot_x_signs", {1.0, 1.0, 1.0, 1.0});
    gait_foot_x_offsets_ =
      declare_parameter<std::vector<double>>("gait_foot_x_offsets", {0.0, 0.0, 0.0, 0.0});
    stand_foot_x_offsets_ =
      declare_parameter<std::vector<double>>("stand_foot_x_offsets", {0.0, 0.0, 0.0, 0.0});
    nominal_haa_angles_ =
      declare_parameter<std::vector<double>>("nominal_haa_angles", {0.0, 0.0, 0.0, 0.0});
    nominal_hfe_angles_ =
      declare_parameter<std::vector<double>>("nominal_hfe_angles", {-0.67, -0.67, -0.67, -0.67});
    nominal_kfe_angles_ =
      declare_parameter<std::vector<double>>("nominal_kfe_angles", {0.09, 0.09, 0.09, 0.09});
    direct_haa_signs_ =
      declare_parameter<std::vector<double>>("direct_haa_signs", {1.0, 1.0, 1.0, 1.0});
    direct_hfe_signs_ =
      declare_parameter<std::vector<double>>("direct_hfe_signs", {1.0, 1.0, 1.0, 1.0});
    direct_kfe_signs_ =
      declare_parameter<std::vector<double>>("direct_kfe_signs", {1.0, 1.0, 1.0, 1.0});
    direct_step_swing_scales_ =
      declare_parameter<std::vector<double>>("direct_step_swing_scales", {1.0, 1.0, 1.0, 1.0});
    direct_step_support_scales_ =
      declare_parameter<std::vector<double>>("direct_step_support_scales", {1.0, 1.0, 1.0, 1.0});
    auto_sequence_forward_hfe_scales_ =
      declare_parameter<std::vector<double>>(
        "auto_sequence_forward_hfe_scales", {1.0, 1.0, 1.0, 1.0});
    auto_sequence_backward_hfe_scales_ =
      declare_parameter<std::vector<double>>(
        "auto_sequence_backward_hfe_scales", {1.0, 1.0, 1.0, 1.0});
    auto_sequence_turn_hfe_scales_ =
      declare_parameter<std::vector<double>>(
        "auto_sequence_turn_hfe_scales", {1.0, 1.0, 1.0, 1.0});
    stand_haa_targets_ =
      declare_parameter<std::vector<double>>("stand_haa_targets", {0.0, 0.0, 0.0, 0.0});
    stand_hfe_targets_ =
      declare_parameter<std::vector<double>>("stand_hfe_targets", {0.10, 0.10, 0.10, 0.10});
    stand_kfe_targets_ =
      declare_parameter<std::vector<double>>("stand_kfe_targets", {-0.16, -0.16, -0.16, -0.16});
    startup_crouch_haa_targets_ = declare_parameter<std::vector<double>>(
      "startup_crouch_haa_targets", {0.0, 0.0, 0.0, 0.0});
    startup_crouch_hfe_targets_ = declare_parameter<std::vector<double>>(
      "startup_crouch_hfe_targets", {0.05, 0.05, 0.05, 0.05});
    startup_crouch_kfe_targets_ = declare_parameter<std::vector<double>>(
      "startup_crouch_kfe_targets", {-0.08, -0.08, -0.08, -0.08});
    const auto thigh_lengths = declare_parameter<std::vector<double>>(
      "thigh_lengths", {0.1993, 0.1991, 0.1948, 0.2021});
    const auto shank_lengths = declare_parameter<std::vector<double>>(
      "shank_lengths", {0.2224, 0.2224, 0.2219, 0.2223});
    const auto gravity_masses =
      declare_parameter<std::vector<double>>("gravity_masses", {0.62, 0.62, 0.62, 0.62});
    auto_zero_offsets_ = declare_parameter<bool>("auto_zero_offsets", true);
    manual_hfe_offsets_ =
      declare_parameter<std::vector<double>>("hfe_offsets", {0.0, 0.0, 0.0, 0.0});
    manual_kfe_offsets_ =
      declare_parameter<std::vector<double>>("kfe_offsets", {0.0, 0.0, 0.0, 0.0});

    haa_kp_ = declare_parameter<double>("haa_kp", 18.0);
    haa_kd_ = declare_parameter<double>("haa_kd", 2.0);
    hfe_kp_ = declare_parameter<double>("hfe_kp", 70.0);
    hfe_kd_ = declare_parameter<double>("hfe_kd", 7.5);
    kfe_kp_ = declare_parameter<double>("kfe_kp", 60.0);
    kfe_kd_ = declare_parameter<double>("kfe_kd", 6.0);

    // 配置文件一旦长度不对，就在启动阶段直接抛错，避免控制循环期
    // 才触发数组越界。
    validate_vector_sizes({
      {"phase_offsets", &phase_offsets_},
      {"foot_x_signs", &foot_x_signs_},
      {"gait_foot_x_offsets", &gait_foot_x_offsets_},
      {"stand_foot_x_offsets", &stand_foot_x_offsets_},
      {"nominal_haa_angles", &nominal_haa_angles_},
      {"nominal_hfe_angles", &nominal_hfe_angles_},
      {"nominal_kfe_angles", &nominal_kfe_angles_},
      {"direct_haa_signs", &direct_haa_signs_},
      {"direct_hfe_signs", &direct_hfe_signs_},
      {"direct_kfe_signs", &direct_kfe_signs_},
      {"direct_step_hfe_signs", &direct_step_hfe_signs_},
      {"direct_step_kfe_signs", &direct_step_kfe_signs_},
      {"direct_step_hfe_swing_scales", &direct_step_hfe_swing_scales_},
      {"direct_step_kfe_swing_scales", &direct_step_kfe_swing_scales_},
      {"direct_step_hfe_support_scales", &direct_step_hfe_support_scales_},
      {"direct_step_kfe_support_scales", &direct_step_kfe_support_scales_},
      {"direct_step_swing_scales", &direct_step_swing_scales_},
      {"direct_step_support_scales", &direct_step_support_scales_},
      {"auto_sequence_forward_hfe_scales", &auto_sequence_forward_hfe_scales_},
      {"auto_sequence_backward_hfe_scales", &auto_sequence_backward_hfe_scales_},
      {"auto_sequence_turn_hfe_scales", &auto_sequence_turn_hfe_scales_},
      {"stand_haa_targets", &stand_haa_targets_},
      {"stand_hfe_targets", &stand_hfe_targets_},
      {"stand_kfe_targets", &stand_kfe_targets_},
      {"startup_crouch_haa_targets", &startup_crouch_haa_targets_},
      {"startup_crouch_hfe_targets", &startup_crouch_hfe_targets_},
      {"startup_crouch_kfe_targets", &startup_crouch_kfe_targets_},
      {"thigh_lengths", &thigh_lengths},
      {"shank_lengths", &shank_lengths},
      {"gravity_masses", &gravity_masses},
      {"hfe_offsets", &manual_hfe_offsets_},
      {"kfe_offsets", &manual_kfe_offsets_},
    });

    // 对容易引发数值问题或物理不合理的参数做基础裁剪。
    // 这一步的目标不是“自动修正错误配置”，而是把参数限制在控制器
    // 能够稳定工作的范围内。
    swing_ratio_ = std::clamp(swing_ratio_, 0.05, 0.95);
    stance_transition_ratio_ = std::clamp(stance_transition_ratio_, 0.0, 0.45);
    walk_start_phase_ = normalize_gait_phase(walk_start_phase_);
    swing_gain_scale_ = std::clamp(swing_gain_scale_, 0.05, 1.0);
    swing_hfe_motion_scale_ = std::clamp(swing_hfe_motion_scale_, 0.0, 1.0);
    swing_extra_kfe_flexion_ = std::max(swing_extra_kfe_flexion_, 0.0);
    direct_step_hfe_amplitude_ = std::clamp(direct_step_hfe_amplitude_, 0.0, 0.50);
    direct_step_kfe_amplitude_ = std::clamp(direct_step_kfe_amplitude_, 0.0, 0.80);
    direct_step_support_hfe_amplitude_ =
      std::clamp(direct_step_support_hfe_amplitude_, 0.0, 0.30);
    direct_step_support_kfe_amplitude_ =
      std::clamp(direct_step_support_kfe_amplitude_, 0.0, 0.50);
    for (double & sign : direct_step_hfe_signs_)
    {
      sign = sign >= 0.0 ? 1.0 : -1.0;
    }
    for (double & sign : direct_step_kfe_signs_)
    {
      sign = sign >= 0.0 ? 1.0 : -1.0;
    }
    for (double & scale : direct_step_swing_scales_)
    {
      scale = std::clamp(scale, 0.0, 2.0);
    }
    for (double & scale : direct_step_hfe_swing_scales_)
    {
      scale = std::clamp(scale, 0.0, 2.0);
    }
    for (double & scale : direct_step_kfe_swing_scales_)
    {
      scale = std::clamp(scale, 0.0, 2.0);
    }
    for (double & scale : direct_step_support_scales_)
    {
      scale = std::clamp(scale, 0.0, 3.0);
    }
    for (double & scale : direct_step_hfe_support_scales_)
    {
      scale = std::clamp(scale, 0.0, 3.0);
    }
    for (double & scale : direct_step_kfe_support_scales_)
    {
      scale = std::clamp(scale, 0.0, 3.0);
    }
    rear_swing_step_length_scale_ = std::clamp(rear_swing_step_length_scale_, 0.1, 2.0);
    rear_swing_step_height_scale_ = std::clamp(rear_swing_step_height_scale_, 0.1, 2.0);
    rear_swing_gain_scale_ = std::clamp(rear_swing_gain_scale_, 0.1, 2.0);
    rear_swing_hfe_motion_scale_ = std::clamp(rear_swing_hfe_motion_scale_, 0.1, 4.0);
    rear_swing_extra_kfe_flexion_scale_ =
      std::clamp(rear_swing_extra_kfe_flexion_scale_, 0.5, 4.0);
    rear_swing_pitch_feedforward_offset_ = std::max(rear_swing_pitch_feedforward_offset_, 0.0);
    rear_swing_body_x_shift_ = std::max(rear_swing_body_x_shift_, 0.0);
    support_pitch_balance_gain_ = std::max(support_pitch_balance_gain_, 0.0);
    support_pitch_balance_max_offset_ = std::max(support_pitch_balance_max_offset_, 0.0);
    gait_period_ = std::max(gait_period_, 0.1);
    control_rate_ = std::max(control_rate_, 1.0);
    startup_settle_duration_ = std::max(startup_settle_duration_, 0.0);
    stand_up_duration_ = std::max(stand_up_duration_, 0.0);
    startup_knee_first_ratio_ = std::clamp(startup_knee_first_ratio_, 0.0, 1.0);
    startup_front_leg_complete_ratio_ =
      std::clamp(startup_front_leg_complete_ratio_, 0.05, 1.0);
    startup_rear_hip_start_ratio_ = std::clamp(startup_rear_hip_start_ratio_, 0.0, 0.95);
    startup_rear_knee_start_ratio_ = std::clamp(startup_rear_knee_start_ratio_, 0.0, 0.95);
    joint_isolation_kfe_target_ = std::clamp(joint_isolation_kfe_target_, kKfeLower, kKfeUpper);
    stand_hold_duration_ = std::max(stand_hold_duration_, 0.0);
    walk_ramp_duration_ = std::max(walk_ramp_duration_, 0.0);
    stand_gain_scale_ = std::max(stand_gain_scale_, 0.0);
    walk_gain_scale_ = std::clamp(walk_gain_scale_, 0.1, 2.5);
    auto_sequence_initial_stand_duration_ =
      std::max(auto_sequence_initial_stand_duration_, 0.0);
      auto_sequence_in_place_step_duration_ =
        std::max(auto_sequence_in_place_step_duration_, 0.0);
      auto_sequence_forward_duration_ = std::max(auto_sequence_forward_duration_, 0.0);
      auto_sequence_backward_duration_ = std::max(auto_sequence_backward_duration_, 0.0);
      auto_sequence_turn_right_duration_ = std::max(auto_sequence_turn_right_duration_, 0.0);
      auto_sequence_turn_left_duration_ = std::max(auto_sequence_turn_left_duration_, 0.0);
      auto_sequence_forward_step_length_ =
        std::clamp(auto_sequence_forward_step_length_, -0.08, 0.08);
      auto_sequence_backward_step_length_ =
        std::clamp(auto_sequence_backward_step_length_, -0.08, 0.08);
      auto_sequence_turn_step_length_ =
        std::clamp(auto_sequence_turn_step_length_, -0.08, 0.08);
      auto_sequence_turn_left_step_length_ =
        std::clamp(auto_sequence_turn_left_step_length_, -0.08, 0.08);
      auto_sequence_forward_step_height_ =
        std::clamp(auto_sequence_forward_step_height_, 0.0, 0.05);
      auto_sequence_forward_hfe_amplitude_ =
        std::clamp(auto_sequence_forward_hfe_amplitude_, 0.0, 0.12);
      auto_sequence_backward_hfe_amplitude_ =
        std::clamp(auto_sequence_backward_hfe_amplitude_, 0.0, 0.12);
      auto_sequence_turn_hfe_amplitude_ =
        std::clamp(auto_sequence_turn_hfe_amplitude_, 0.0, 0.12);
      for (double & scale : auto_sequence_forward_hfe_scales_)
      {
        scale = std::clamp(scale, 0.0, 2.0);
      }
      for (double & scale : auto_sequence_backward_hfe_scales_)
      {
        scale = std::clamp(scale, 0.0, 2.0);
      }
      for (double & scale : auto_sequence_turn_hfe_scales_)
      {
        scale = std::clamp(scale, 0.0, 2.0);
      }
    startup_min_support_scale_ = std::clamp(startup_min_support_scale_, 0.0, 1.0);
    startup_initial_gain_scale_ = std::clamp(startup_initial_gain_scale_, 0.0, stand_gain_scale_);
    stand_haa_slew_rate_ = std::max(stand_haa_slew_rate_, 0.0);
    stand_hfe_slew_rate_ = std::max(stand_hfe_slew_rate_, 0.0);
    stand_kfe_slew_rate_ = std::max(stand_kfe_slew_rate_, 0.0);
    gravity_ff_scale_ = std::max(gravity_ff_scale_, 0.0);
    shank_mass_ratio_ = std::clamp(shank_mass_ratio_, 0.0, 1.0);
    debug_log_period_ = std::max(debug_log_period_, 0.1);
    swing_period_ = gait_period_ * swing_ratio_;
    stance_period_ = gait_period_ * (1.0 - swing_ratio_);
    if (gait_profile_ != "legacy_walk" && gait_profile_ != "trot")
    {
      RCLCPP_WARN(
        get_logger(),
        "Unknown gait_profile '%s'. Falling back to legacy_walk.",
        gait_profile_.c_str());
      gait_profile_ = "legacy_walk";
    }
    if (use_trot_profile() && (swing_ratio_ < 0.35 || swing_ratio_ > 0.65))
    {
      RCLCPP_WARN(
        get_logger(),
        "trot profile works best with swing_ratio near 0.5, current value=%.3f",
        swing_ratio_);
    }
    if (enable_startup_stand_)
    {
      motion_state_ = MotionState::kStandUp;
    }
    else
    {
      motion_state_ = stand_only_ ? MotionState::kStandHold : MotionState::kWalk;
    }

    // 初始化每条腿的几何、滤波器、重力等效质量和零位偏置。
    // 如果启用了 auto_zero_offsets_，则根据名义站姿和 IK 自动推算
    // 偏置；否则直接使用配置里的手工偏置。
    for (size_t i = 0; i < kLegCount; ++i)
    {
      geometries_[i] = {thigh_lengths[i], shank_lengths[i]};
      leg_states_[i].hfe_filter.set_alpha(filter_alpha);
      leg_states_[i].kfe_filter.set_alpha(filter_alpha);
      leg_states_[i].hfe_velocity_filter.set_alpha(filter_alpha);
      leg_states_[i].kfe_velocity_filter.set_alpha(filter_alpha);
      const double total_mass = std::max(gravity_masses[i], 0.0);
      leg_states_[i].shank_mass = total_mass * shank_mass_ratio_;
      leg_states_[i].thigh_mass = total_mass - leg_states_[i].shank_mass;

      if (auto_zero_offsets_)
      {
        const double stand_x = foot_x_signs_[i] * stand_foot_x_offsets_[i];
        const IKResult ik0 = inverse_kinematics(geometries_[i], stand_x, stance_depth_);
        leg_states_[i].hfe_offset = nominal_hfe_angles_[i] - ik0.hfe;
        leg_states_[i].kfe_offset = nominal_kfe_angles_[i] - ik0.kfe;
      }
      else
      {
        leg_states_[i].hfe_offset = manual_hfe_offsets_[i];
        leg_states_[i].kfe_offset = manual_kfe_offsets_[i];
      }
    }

    // 如果启用了“启动低趴姿态”，则在收到实际关节状态之前先预置一组
    // 初始命令，避免刚上电时关节目标为空。
    if (use_startup_crouch_pose_ && use_direct_joint_stand_targets_ && !joint_isolation_kfe_only())
    {
      initialize_startup_crouch_positions();
    }

    // 通信拓扑很简单:
    // - 订阅 /joint_states
    // - 周期发布 /motor_cmd
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&QuadrupedGaitController::joint_state_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&QuadrupedGaitController::cmd_vel_callback, this, std::placeholders::_1));
    gait_action_sub_ = create_subscription<std_msgs::msg::String>(
      gait_action_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&QuadrupedGaitController::gait_action_callback, this, std::placeholders::_1));

    const double dt = 1.0 / control_rate_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&QuadrupedGaitController::control_loop, this));

    RCLCPP_INFO(get_logger(), "Quadruped gait controller started.");
    RCLCPP_INFO(get_logger(), "Initial motion state: %s", motion_state_name(motion_state_));
    RCLCPP_INFO(get_logger(), "Gait profile: %s", gait_profile_.c_str());
    RCLCPP_INFO(get_logger(), "Gait action topic: %s", gait_action_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "allow_partial_joint_state_startup: %s",
      allow_partial_joint_state_startup_ ? "true" : "false");
    if (enable_auto_sequence_)
      {
        RCLCPP_INFO(
          get_logger(),
          "Auto sequence enabled: stand %.2fs -> in_place_step %.2fs -> "
          "forward %.2fs -> backward %.2fs -> turn_right %.2fs -> turn_left %.2fs -> stand",
          auto_sequence_initial_stand_duration_,
          auto_sequence_in_place_step_duration_,
          auto_sequence_forward_duration_,
          auto_sequence_backward_duration_,
          auto_sequence_turn_right_duration_,
          auto_sequence_turn_left_duration_);
      }
  }

private:
  // 主控制循环。
  //
  // 每一轮循环都做以下事情:
  // 1. 更新时间步长并推进状态机
  // 2. 根据当前相位生成四条腿的目标足端轨迹
  // 3. 计算站立目标与步态目标，并按当前阶段平滑混合
  // 4. 为每个关节写入 MIT 风格命令
  void control_loop()
  {
    const rclcpp::Time now = this->now();
    double dt = 0.0;

    // 用 ROS time 计算本轮控制实际步长。第一次进入时没有历史时间，
    // 因此把 dt 视作 0。
    if (!last_time_initialized_)
    {
      last_time_ = now;
      last_time_initialized_ = true;
    }
    else
    {
      dt = (now - last_time_).seconds();
      last_time_ = now;
    }

    dt = std::clamp(dt, 0.0, 0.1);

    // cmd_vel 超时检测: 若长时间未收到新消息则归零，防止机器狗失控漂移。
    if (cmd_vel_received_) {
      const double vel_age = (now - last_cmd_vel_time_).seconds();
      if (vel_age > cmd_vel_timeout_) {
        cmd_vel_linear_x_ = 0.0;
        cmd_vel_angular_z_ = 0.0;
      }
    }
    const double cmd_speed = std::abs(cmd_vel_linear_x_) + std::abs(cmd_vel_angular_z_);
    teleop_walk_requested_ = cmd_vel_received_ && (cmd_speed > cmd_vel_deadzone_);

    // 如果还没捕获到完整的实际关节姿态，站立阶段的插值起点就无法确定，
    // 因此优先尝试抓取一次”真实起始姿态”。
    maybe_capture_stand_start_positions();
    update_motion_state(dt);
    const bool trot_profile = use_trot_profile();

    // 这些缩放量把“状态机阶段”转成对控制输出的连续调制:
    // - support_scale: 启动站立时逐步恢复支撑/前馈
    // - gait_blend:    从纯站姿平滑过渡到步态轨迹
    // - gain_scale:    从较软的启动增益逐渐抬到正常增益
    const double support_scale = current_support_scale();
    const double gait_blend = current_gait_blend();
    const double gain_scale = current_gain_scale();
    const double stand_hip_alpha = current_stand_hip_alpha();
    const double stand_knee_alpha = current_stand_knee_alpha();

    // 每条腿的相位、摆动/支撑状态和局部 tau 统一由 gait schedule 生成。
    // 这样后续扩展 crawl/pace/bound 时，主控制循环不需要继续堆相位细节。
    const auto gait_schedule = make_gait_schedule(global_phase_, phase_offsets_, swing_ratio_);
    double direct_step_support_alpha = 0.0;
    if (use_direct_joint_in_place_step_ && motion_state_ == MotionState::kWalk)
    {
      for (const auto & phase_state : gait_schedule)
      {
        if (phase_state.in_swing)
        {
          direct_step_support_alpha =
            std::max(direct_step_support_alpha, std::sin(kPi * phase_state.swing_tau));
        }
      }
    }
    const bool rear_leg_in_swing =
      !trot_profile && (gait_schedule[1].in_swing || gait_schedule[3].in_swing);

    double front_support_y_sum = 0.0;
    double rear_support_y_sum = 0.0;
    size_t front_support_count = 0;
    size_t rear_support_count = 0;
    double front_all_y_sum = 0.0;
    double rear_all_y_sum = 0.0;
    size_t front_all_count = 0;
    size_t rear_all_count = 0;

    // 利用实际关节角做一次正运动学，估计前后腿当前的支撑高度。
    // 这一步不是为了闭环定位足端，而是为了做一个轻量的“俯仰平衡”
    // 补偿，让机身前后不要明显一头高一头低。
    for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
      const auto indices = joint_indices(leg_index);
      if (!actual_position_valid_[indices.hfe] || !actual_position_valid_[indices.kfe])
      {
        continue;
      }

      const auto & leg = leg_states_[leg_index];
      const LegPose2D actual_pose = forward_kinematics(
        geometries_[leg_index],
        actual_positions_[indices.hfe] - leg.hfe_offset,
        actual_positions_[indices.kfe] - leg.kfe_offset);

      if (is_front_leg(leg_index))
      {
        front_all_y_sum += actual_pose.y;
        ++front_all_count;
        if (gait_schedule[leg_index].contact_weight > 0.5)
        {
          front_support_y_sum += actual_pose.y;
          ++front_support_count;
        }
      }
      else
      {
        rear_all_y_sum += actual_pose.y;
        ++rear_all_count;
        if (gait_schedule[leg_index].contact_weight > 0.5)
        {
          rear_support_y_sum += actual_pose.y;
          ++rear_support_count;
        }
      }
    }

    const double front_reference_y = front_support_count > 0 ?
      (front_support_y_sum / static_cast<double>(front_support_count)) :
      (front_all_count > 0 ? (front_all_y_sum / static_cast<double>(front_all_count)) : stance_depth_);
    const double rear_reference_y = rear_support_count > 0 ?
      (rear_support_y_sum / static_cast<double>(rear_support_count)) :
      (rear_all_count > 0 ? (rear_all_y_sum / static_cast<double>(rear_all_count)) : stance_depth_);

    double support_pitch_balance_offset = 0.0;

    // 仅在 walk 态下启用支撑俯仰补偿。补偿量本质上是“前后平均腿长差”
    // 映射成对前腿/后腿足端 y 目标的微调。
    if (motion_state_ == MotionState::kWalk && support_pitch_balance_max_offset_ > 1e-6)
    {
      const double front_minus_rear_y =
        front_reference_y - rear_reference_y - support_pitch_balance_target_delta_;
      support_pitch_balance_offset = std::clamp(
        support_pitch_balance_gain_ * front_minus_rear_y,
        -support_pitch_balance_max_offset_,
        support_pitch_balance_max_offset_);
    }
    if (!trot_profile && motion_state_ == MotionState::kWalk && rear_leg_in_swing)
    {
      support_pitch_balance_offset += rear_swing_pitch_feedforward_offset_;
      support_pitch_balance_offset = std::clamp(
        support_pitch_balance_offset,
        -support_pitch_balance_max_offset_,
        support_pitch_balance_max_offset_);
    }

    // 将 cmd_vel 或自动动作序列映射为步长缩放比例，供逐腿循环使用。
    // 自动序列里的前进/后退/转向共用 direct_joint 前后摆，只改变方向和幅度。
    const bool sequence_motion_active = auto_sequence_motion_active();
    const double sequence_motion_alpha = current_auto_sequence_motion_alpha();
    const double base_step_length =
      sequence_motion_active ? std::abs(current_auto_sequence_step_length()) * sequence_motion_alpha :
      step_length_;
    const double base_step_height =
      sequence_motion_active ? auto_sequence_forward_step_height_ * sequence_motion_alpha :
      step_height_;

    const bool use_cmd_vel_for_stride =
      cmd_vel_received_ && !enable_auto_sequence_ && !action_command_active_;
    const double cmd_linear_scale =
      use_cmd_vel_for_stride ?
      std::clamp(cmd_vel_linear_x_ / std::max(max_linear_vel_, 1e-6), -1.0, 1.0) : 1.0;
    const double cmd_turn_scale =
      use_cmd_vel_for_stride ?
      std::clamp(cmd_vel_angular_z_ / std::max(max_angular_vel_, 1e-6), -1.0, 1.0) : 0.0;

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kAggregateCommandSize, 0.0);

    // 逐腿合成命令。这里是整份控制逻辑最密集的部分:
    // 足端轨迹 -> IK -> 关节目标 -> 站立/步态混合 -> MIT 命令输出
    for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
      const auto indices = joint_indices(leg_index);
      const LegPhaseState phase_state = gait_schedule[leg_index];
      const bool leg_in_swing = phase_state.in_swing;
      const bool rear_leg = !is_front_leg(leg_index);
      const bool hold_startup_crouch_target = use_startup_crouch_pose_ && !enable_startup_stand_;
      const bool use_direct_joint_step =
        use_direct_joint_in_place_step_ &&
        (!sequence_motion_active || auto_sequence_forward_use_direct_step_);

      auto & leg = leg_states_[leg_index];

      // stand_targets 表示“如果当前只想站稳，这条腿应该去哪里”；
      // gait_*     表示“按照步态轨迹，这条腿应该去哪里”。
      // 后面会根据 gait_blend 在两者之间连续过渡，而不是硬切换。
      const JointTargets stand_targets =
        stand_targets_for_leg(leg_index, leg, hold_startup_crouch_target);
      const JointTargets stand_start_targets = current_stand_start_targets(indices);
      double leg_hip_alpha = stand_hip_alpha;
      double leg_knee_alpha = stand_knee_alpha;
      if (startup_front_legs_first_ && motion_state_ == MotionState::kStandUp)
      {
        const double stand_phase = current_stand_phase();
        if (is_front_leg(leg_index))
        {
          const double front_alpha =
            startup_phase_window_alpha(stand_phase, 0.0, startup_front_leg_complete_ratio_);
          leg_hip_alpha = front_alpha;
          leg_knee_alpha = front_alpha;
        }
        else
        {
          leg_hip_alpha =
            startup_phase_window_alpha(stand_phase, startup_rear_hip_start_ratio_, 1.0);
          leg_knee_alpha =
            startup_phase_window_alpha(stand_phase, startup_rear_knee_start_ratio_, 1.0);
        }
      }
      double stand_haa = interpolate(stand_start_targets.haa, stand_targets.haa, leg_hip_alpha);
      double stand_hfe = interpolate(stand_start_targets.hfe, stand_targets.hfe, leg_hip_alpha);
      double stand_kfe = interpolate(stand_start_targets.kfe, stand_targets.kfe, leg_knee_alpha);
      if (joint_isolation_kfe_only())
      {
        stand_haa = stand_start_targets.haa;
        stand_hfe = stand_start_targets.hfe;
        stand_kfe = interpolate(
          stand_start_targets.kfe, joint_isolation_kfe_target_, leg_knee_alpha);
      }

      double gait_haa = stand_targets.haa;
      double gait_hfe = stand_targets.hfe;
      double gait_kfe = stand_targets.kfe;
      double gait_hfe_velocity = 0.0;
      double gait_kfe_velocity = 0.0;

      // direct_joint 真机模式直接围绕 stand_*_targets 调制关节角。
      // 这种情况下跳过足端轨迹、IK 和速度映射，避免每个控制周期做无效计算。
      if (!use_direct_joint_step)
      {
        // Teleop 差速: 左腿(0,1)与右腿(2,3)施加相反的转向分量，实现原地/行进转弯。
        // angular.z > 0 为 ROS 左转(CCW): 右腿多走、左腿少走。
        // 未收到 cmd_vel 时，teleop_amp=1 / local_foot_x_sign=foot_x_signs_ 保持原始配置。
        const bool left_leg = (leg_index == 0 || leg_index == 1);
        double local_foot_x_sign = foot_x_signs_[leg_index];
        double teleop_amp = 1.0;
        if (sequence_motion_active)
        {
          local_foot_x_sign *= current_auto_sequence_stride_direction(leg_index);
        }
        if (use_cmd_vel_for_stride)
        {
          const double turn_comp = left_leg ? -cmd_turn_scale : cmd_turn_scale;
          const double combined = cmd_linear_scale + turn_comp;
          teleop_amp = std::min(std::abs(combined), 1.0);
          local_foot_x_sign = foot_x_signs_[leg_index] * ((combined >= 0.0) ? 1.0 : -1.0);
        }

        const double leg_step_length =
          base_step_length * teleop_amp * (rear_leg ? rear_swing_step_length_scale_ : 1.0);
        const double leg_step_height =
          base_step_height * (rear_leg ? rear_swing_step_height_scale_ : 1.0);
        const double gait_foot_x_offset =
          foot_x_signs_[leg_index] * gait_foot_x_offsets_[leg_index];
        const double gait_pitch_y_offset =
          is_front_leg(leg_index) ? -support_pitch_balance_offset : support_pitch_balance_offset;
        const double gait_body_x_shift =
          (!trot_profile && motion_state_ == MotionState::kWalk &&
          rear_leg_in_swing && !leg_in_swing) ?
          -foot_x_signs_[leg_index] * rear_swing_body_x_shift_ :
          0.0;

        double foot_x = 0.0;
        double foot_y = stance_depth_ + gait_pitch_y_offset;
        double foot_vx = 0.0;
        double foot_vy = 0.0;

        // 摆动相和支撑相使用不同轨迹:
        // - 摆动相强调抬脚和平滑落地
        // - 支撑相强调机体相对足端的推进/回拉
        if (leg_in_swing)
        {
          const FootState trajectory = trajectory_cycloid(
            phase_state.swing_tau,
            leg_step_length,
            leg_step_height,
            -leg_step_length / 2.0,
            swing_period_);
          foot_x = local_foot_x_sign * trajectory.x + gait_foot_x_offset;
          foot_y = stance_depth_ + gait_pitch_y_offset - trajectory.y;
          foot_vx = local_foot_x_sign * trajectory.vx;
          foot_vy = -trajectory.vy;
        }
        else
        {
          if (trot_profile)
          {
            const FootState trajectory = trajectory_stance_trot(
              phase_state.stance_tau, leg_step_length, stance_period_, stance_transition_ratio_);
            foot_x =
              local_foot_x_sign * trajectory.x +
              gait_foot_x_offset + gait_body_x_shift;
            foot_vx = local_foot_x_sign * trajectory.vx;
          }
          else
          {
            const double front_x = leg_step_length / 2.0;
            const double back_x = -leg_step_length / 2.0;
            foot_x =
              local_foot_x_sign * (front_x + (back_x - front_x) * phase_state.stance_tau) +
              gait_foot_x_offset + gait_body_x_shift;
            foot_vx = local_foot_x_sign * ((back_x - front_x) / stance_period_);
          }
          foot_y = stance_depth_ + gait_pitch_y_offset;
          foot_vy = 0.0;
        }

        const IKResult ik = inverse_kinematics(geometries_[leg_index], foot_x, foot_y);
        const JointVel joint_velocity = compute_joint_velocity(
          geometries_[leg_index], foot_x, foot_y, foot_vx, foot_vy);

        gait_haa = std::clamp(nominal_haa_angles_[leg_index], kHaaLower, kHaaUpper);
        gait_hfe = std::clamp(
          leg.hfe_filter.filter(ik.hfe + leg.hfe_offset), kHfeLower, kHfeUpper);
        gait_kfe = std::clamp(
          leg.kfe_filter.filter(ik.kfe + leg.kfe_offset), kKfeLower, kKfeUpper);
        gait_hfe_velocity = leg.hfe_velocity_filter.filter(joint_velocity.hfe);
        gait_kfe_velocity = leg.kfe_velocity_filter.filter(joint_velocity.kfe);
      }

      // 真机原地踏步可以直接围绕已标定好的站姿关节角做小幅收腿。
      // 这样不会在进入 walk 态时从 direct stand target 切到另一套 IK 基准姿态。
      if (use_direct_joint_step)
      {
        double direct_base_hfe = stand_targets.hfe;
        double direct_base_kfe = stand_targets.kfe;
        double direct_base_hfe_velocity = 0.0;
        double direct_base_kfe_velocity = 0.0;

        if (sequence_motion_active && auto_sequence_forward_use_direct_step_)
        {
          const double foot_x_direction =
            direct_step_forward_direction(
              leg_index, leg, stand_targets.hfe, stand_targets.kfe) *
            foot_x_signs_[leg_index] *
            current_auto_sequence_stride_direction(leg_index);
          double stride_profile = 0.0;
          double stride_velocity = 0.0;
          if (leg_in_swing)
          {
            stride_profile = -std::cos(kPi * phase_state.swing_tau);
            stride_velocity =
              swing_period_ > 1e-6 ?
              kPi * std::sin(kPi * phase_state.swing_tau) / swing_period_ :
              0.0;
          }
          else
          {
            stride_profile = 1.0 - 2.0 * phase_state.stance_tau;
            stride_velocity = stance_period_ > 1e-6 ? -2.0 / stance_period_ : 0.0;
          }

          const double forward_hfe_amplitude =
            current_auto_sequence_hfe_amplitude(leg_index) *
            sequence_motion_alpha;
          direct_base_hfe += foot_x_direction * forward_hfe_amplitude * stride_profile;
          direct_base_hfe_velocity += foot_x_direction * forward_hfe_amplitude * stride_velocity;
        }

        gait_haa = stand_targets.haa;
        gait_hfe = std::clamp(direct_base_hfe, kHfeLower, kHfeUpper);
        gait_kfe = std::clamp(direct_base_kfe, kKfeLower, kKfeUpper);
        gait_hfe_velocity = direct_base_hfe_velocity;
        gait_kfe_velocity = direct_base_kfe_velocity;

        if (motion_state_ == MotionState::kWalk && leg_in_swing)
        {
          const double base_swing_scale = direct_step_swing_scales_[leg_index];
          const double hfe_swing_scale =
            base_swing_scale * direct_step_hfe_swing_scales_[leg_index];
          const double kfe_swing_scale =
            base_swing_scale * direct_step_kfe_swing_scales_[leg_index];
          const double swing_lift = std::sin(kPi * phase_state.swing_tau);
          const double lift_velocity =
            swing_period_ > 1e-6 ?
            (kPi * std::cos(kPi * phase_state.swing_tau) / swing_period_) :
            0.0;
          const double hfe_direction = direct_step_lift_direction(
            leg_index, leg, stand_targets.hfe, stand_targets.kfe, kHfeJointIndex) *
            direct_step_hfe_signs_[leg_index];
          const double kfe_direction = direct_step_lift_direction(
            leg_index, leg, stand_targets.hfe, stand_targets.kfe, kKfeJointIndex) *
            direct_step_kfe_signs_[leg_index];

          gait_hfe = std::clamp(
            direct_base_hfe +
            hfe_direction * direct_step_hfe_amplitude_ * hfe_swing_scale * swing_lift,
            kHfeLower,
            kHfeUpper);
          gait_kfe = std::clamp(
            direct_base_kfe +
            kfe_direction * direct_step_kfe_amplitude_ * kfe_swing_scale * swing_lift,
            kKfeLower,
            kKfeUpper);
          gait_hfe_velocity =
            direct_base_hfe_velocity +
            hfe_direction * direct_step_hfe_amplitude_ * hfe_swing_scale * lift_velocity;
          gait_kfe_velocity =
            direct_base_kfe_velocity +
            kfe_direction * direct_step_kfe_amplitude_ * kfe_swing_scale * lift_velocity;
        }
        else if (motion_state_ == MotionState::kWalk && direct_step_support_alpha > 1e-6)
        {
          const double base_support_scale = direct_step_support_scales_[leg_index];
          const double hfe_support_scale =
            base_support_scale * direct_step_hfe_support_scales_[leg_index];
          const double kfe_support_scale =
            base_support_scale * direct_step_kfe_support_scales_[leg_index];
          const double hfe_direction = -direct_step_lift_direction(
            leg_index, leg, stand_targets.hfe, stand_targets.kfe, kHfeJointIndex) *
            direct_step_hfe_signs_[leg_index];
          const double kfe_direction = -direct_step_lift_direction(
            leg_index, leg, stand_targets.hfe, stand_targets.kfe, kKfeJointIndex) *
            direct_step_kfe_signs_[leg_index];

          gait_hfe = std::clamp(
            direct_base_hfe +
            hfe_direction * direct_step_support_hfe_amplitude_ * hfe_support_scale *
            direct_step_support_alpha,
            kHfeLower,
            kHfeUpper);
          gait_kfe = std::clamp(
            direct_base_kfe +
            kfe_direction * direct_step_support_kfe_amplitude_ * kfe_support_scale *
            direct_step_support_alpha,
            kKfeLower,
            kKfeUpper);
        }
      }
      // 摆动相时，允许对 HFE/KFE 做额外调制:
      // - HFE 可以更保守或更激进地朝轨迹目标摆动
      // - KFE 可以额外加一段“卷膝”动作，帮助离地
      else if (motion_state_ == MotionState::kWalk && leg_in_swing)
      {
        const double swing_tau = phase_state.swing_tau;
        const double leg_swing_hfe_motion_scale =
          swing_hfe_motion_scale_ * (rear_leg ? rear_swing_hfe_motion_scale_ : 1.0);
        const double leg_swing_extra_kfe_flexion =
          swing_extra_kfe_flexion_ *
          (rear_leg ? rear_swing_extra_kfe_flexion_scale_ : 1.0);
        const double knee_flex_alpha = std::sin(kPi * swing_tau);
        const double extra_kfe_velocity =
          swing_period_ > 1e-6 ?
          (leg_swing_extra_kfe_flexion * kPi * std::cos(kPi * swing_tau) / swing_period_) :
          0.0;

        gait_hfe = std::clamp(
          interpolate(stand_targets.hfe, gait_hfe, leg_swing_hfe_motion_scale),
          kHfeLower, kHfeUpper);
        gait_kfe = std::clamp(
          gait_kfe + leg_swing_extra_kfe_flexion * knee_flex_alpha,
          kKfeLower, kKfeUpper);
        gait_hfe_velocity *= leg_swing_hfe_motion_scale;
        gait_kfe_velocity += extra_kfe_velocity;
      }

      const double stand_thigh_ff = gravity_ff_scale_ * compute_thigh_feedforward(
        leg_index, stand_hfe, stand_kfe);
      const double gait_thigh_ff = gravity_ff_scale_ * compute_thigh_feedforward(
        leg_index, gait_hfe, gait_kfe);
      const double stand_knee_ff = gravity_ff_scale_ * compute_knee_feedforward(
        leg_index, stand_hfe, stand_kfe);
      const double gait_knee_ff = gravity_ff_scale_ * compute_knee_feedforward(
        leg_index, gait_hfe, gait_kfe);
      const double raw_cmd_haa = interpolate(stand_haa, gait_haa, gait_blend);
      const double raw_cmd_hfe = interpolate(stand_hfe, gait_hfe, gait_blend);
      const double raw_cmd_kfe = interpolate(stand_kfe, gait_kfe, gait_blend);

      // 站立阶段开启关节目标变化率限制，避免启动抬身或切换站姿时命令突变。
      // 一旦进入 walk 态，就直接使用目标值，不再额外限速。
      const double cmd_haa =
        apply_command_slew_limit(indices.haa, kHaaJointIndex, raw_cmd_haa, dt);
      const double cmd_hfe =
        apply_command_slew_limit(indices.hfe, kHfeJointIndex, raw_cmd_hfe, dt);
      const double cmd_kfe =
        apply_command_slew_limit(indices.kfe, kKfeJointIndex, raw_cmd_kfe, dt);
      const double cmd_hfe_velocity = gait_blend * gait_hfe_velocity;
      const double cmd_kfe_velocity = gait_blend * gait_kfe_velocity;
      const double thigh_ff = support_scale * interpolate(stand_thigh_ff, gait_thigh_ff, gait_blend);
      const double knee_ff = support_scale * interpolate(stand_knee_ff, gait_knee_ff, gait_blend);
      const double leg_gain_scale =
        (motion_state_ == MotionState::kWalk && leg_in_swing) ?
        swing_gain_scale_ * (rear_leg ? rear_swing_gain_scale_ : 1.0) : 1.0;
      const double effective_gain_scale = support_scale * gain_scale * leg_gain_scale;

      // 最终输出采用 MIT 风格五元组:
      // [position, velocity, torque_feedforward, kp, kd]
      // HAA 当前只做位置-阻尼控制；HFE/KFE 则叠加速度目标和重力前馈。
      write_joint_command(
        msg.data,
        leg_index,
        kHaaJointIndex,
        cmd_haa,
        0.0,
        0.0,
        effective_gain_scale * haa_kp_,
        effective_gain_scale * haa_kd_);
      write_joint_command(
        msg.data,
        leg_index,
        kHfeJointIndex,
        cmd_hfe,
        cmd_hfe_velocity,
        thigh_ff,
        effective_gain_scale * hfe_kp_,
        effective_gain_scale * hfe_kd_);
      write_joint_command(
        msg.data,
        leg_index,
        kKfeJointIndex,
        cmd_kfe,
        cmd_kfe_velocity,
        knee_ff,
        effective_gain_scale * kfe_kp_,
        effective_gain_scale * kfe_kd_);

      set_leg_joint_values(commanded_positions_, indices, {cmd_haa, cmd_hfe, cmd_kfe});
    }

    // 发布整机命令，并在 walk 态下推进全局步态相位。
    cmd_pub_->publish(msg);
    maybe_log_joint_tracking(now);

    if (motion_state_ == MotionState::kWalk)
    {
      global_phase_ = normalize_gait_phase(global_phase_ + dt / gait_period_);
    }
  }

  // 关节状态回调只消费 position 字段。
  // 对当前控制器来说，velocity 是通过轨迹速度和局部几何解析得到的，
  // 因而这里暂时不依赖 joint_states 里的速度。
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const size_t count = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < count; ++i)
    {
      const auto it = joint_name_to_index_.find(msg->name[i]);
      if (it == joint_name_to_index_.end())
      {
        continue;
      }

      actual_positions_[it->second] = msg->position[i];
      actual_position_valid_[it->second] = true;
    }

    joint_state_received_ = true;
  }

  // cmd_vel 回调: 缓存最新的线速度和角速度，并记录时间戳用于超时检测。
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_vel_linear_x_ = msg->linear.x;
    cmd_vel_angular_z_ = msg->angular.z;
    last_cmd_vel_time_ = this->now();
    cmd_vel_received_ = true;
  }

  void gait_action_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string action = normalize_action_name(msg->data);
    if (action == "auto" || action == "sequence" || action == "auto_sequence")
    {
      action_command_active_ = false;
      enable_auto_sequence_ = true;
      transition_auto_sequence_stage(AutoSequenceStage::kInactive);
      RCLCPP_INFO(get_logger(), "Gait action command -> auto_sequence");
      return;
    }

    AutoSequenceStage next_stage = AutoSequenceStage::kFinalStand;
    if (!parse_gait_action(action, next_stage))
    {
      RCLCPP_WARN(
        get_logger(),
        "Unknown gait action '%s'. Use one of: stand, step, forward, backward, turn_left, turn_right, auto.",
        msg->data.c_str());
      return;
    }

    const bool stage_changed =
      !action_command_active_ || action_command_stage_ != next_stage;
    action_command_active_ = true;
    action_command_stage_ = next_stage;
    enable_auto_sequence_ = false;
    if (stage_changed)
    {
      RCLCPP_INFO(
        get_logger(), "Gait action command -> %s",
        gait_action_stage_name(action_command_stage_));
    }
    apply_gait_action_command(stage_changed);
  }

  // 预留调试钩子。当前实现为空，但保留这个函数可以让后续接入周期性
  // 关节跟踪日志时，不需要改动主循环结构。
  void maybe_log_joint_tracking(const rclcpp::Time & now)
  {
    (void) now;
  }

  // 当前只显式支持 legacy_walk 和 trot 两种步态，其中 trot 会启用
  // 专门的支撑相轨迹与参数解释。
  bool use_trot_profile() const
  {
    return gait_profile_ == "trot";
  }

  // 站立状态机更新逻辑。
  // 状态转移非常直接，但通过 current_*_alpha() 族函数把离散状态转成
  // 连续插值权重，从而让站起和行走切换都保持平滑。
  void update_motion_state(double dt)
  {
    if (motion_state_ == MotionState::kStandUp && !startup_pose_initialized())
    {
      return;
    }

    state_elapsed_ += dt;
    if (update_auto_sequence(dt))
    {
      return;
    }

    if (action_command_active_)
    {
      update_gait_action_command_state();
      return;
    }

    switch (motion_state_)
    {
      case MotionState::kStandUp:
        if (state_elapsed_ >= startup_settle_duration_ + stand_up_duration_)
        {
          if (stand_only_ || stand_hold_duration_ > 0.0)
          {
            transition_to(MotionState::kStandHold);
          }
          else
          {
            transition_to(MotionState::kWalk);
          }
        }
        break;
      case MotionState::kStandHold:
        startup_complete_ = true;
        if (!stand_only_)
        {
          // 未接入手柄: 按原逻辑在 hold_duration 后进入行走
          if (!cmd_vel_received_ && state_elapsed_ >= stand_hold_duration_)
          {
            transition_to(MotionState::kWalk);
          }
          // 已接入手柄: 只要有非零速度指令就立即开始行走
          else if (cmd_vel_received_ && teleop_walk_requested_)
          {
            transition_to(MotionState::kWalk);
          }
        }
        break;
      case MotionState::kWalk:
        // 已接入手柄且速度归零: 切回站立，等待下一次行走指令
        if (cmd_vel_received_ && startup_complete_ && !teleop_walk_requested_)
        {
          transition_to(MotionState::kStandHold);
        }
        break;
    }
  }

  bool update_auto_sequence(double dt)
  {
    if (action_command_active_ || !enable_auto_sequence_)
    {
      return false;
    }

    if (auto_sequence_stage_ == AutoSequenceStage::kInactive)
    {
      transition_auto_sequence_stage(AutoSequenceStage::kInitialStand);
    }

    if (motion_state_ == MotionState::kStandUp)
    {
      if (state_elapsed_ >= startup_settle_duration_ + stand_up_duration_)
      {
        transition_to(MotionState::kStandHold);
        transition_auto_sequence_stage(AutoSequenceStage::kInitialStand);
      }
      return true;
    }

    auto_sequence_elapsed_ += std::max(dt, 0.0);

    switch (auto_sequence_stage_)
    {
      case AutoSequenceStage::kInitialStand:
        if (motion_state_ != MotionState::kStandHold)
        {
          transition_to(MotionState::kStandHold);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_initial_stand_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kInPlaceStep);
          transition_to(MotionState::kWalk);
        }
        return true;

      case AutoSequenceStage::kInPlaceStep:
        if (motion_state_ != MotionState::kWalk)
        {
          transition_to(MotionState::kWalk);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_in_place_step_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kForward);
          restart_auto_sequence_walk_stage();
        }
        return true;

      case AutoSequenceStage::kForward:
        if (motion_state_ != MotionState::kWalk)
        {
          transition_to(MotionState::kWalk);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_forward_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kBackward);
          restart_auto_sequence_walk_stage();
        }
        return true;

      case AutoSequenceStage::kBackward:
        if (motion_state_ != MotionState::kWalk)
        {
          transition_to(MotionState::kWalk);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_backward_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kTurnRight);
          restart_auto_sequence_walk_stage();
        }
        return true;

      case AutoSequenceStage::kTurnRight:
        if (motion_state_ != MotionState::kWalk)
        {
          transition_to(MotionState::kWalk);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_turn_right_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kTurnLeft);
          restart_auto_sequence_walk_stage();
        }
        return true;

      case AutoSequenceStage::kTurnLeft:
        if (motion_state_ != MotionState::kWalk)
        {
          transition_to(MotionState::kWalk);
        }
        if (auto_sequence_elapsed_ >= auto_sequence_turn_left_duration_)
        {
          transition_auto_sequence_stage(AutoSequenceStage::kFinalStand);
          transition_to(MotionState::kStandHold);
        }
        return true;

      case AutoSequenceStage::kFinalStand:
        if (motion_state_ != MotionState::kStandHold)
        {
          transition_to(MotionState::kStandHold);
        }
        return true;

      case AutoSequenceStage::kInactive:
        return true;
    }

    return true;
  }

  void update_gait_action_command_state()
  {
    if (motion_state_ == MotionState::kStandUp)
    {
      if (state_elapsed_ >= startup_settle_duration_ + stand_up_duration_)
      {
        transition_to(gait_action_requests_walk(action_command_stage_) ?
          MotionState::kWalk : MotionState::kStandHold);
      }
      return;
    }

    apply_gait_action_command(false);
  }

  void apply_gait_action_command(bool reset_walk_stage)
  {
    if (!action_command_active_ || motion_state_ == MotionState::kStandUp)
    {
      return;
    }

    if (gait_action_requests_walk(action_command_stage_))
    {
      if (motion_state_ == MotionState::kWalk)
      {
        if (reset_walk_stage)
        {
          restart_auto_sequence_walk_stage();
        }
      }
      else
      {
        transition_to(MotionState::kWalk);
      }
      return;
    }

    if (motion_state_ != MotionState::kStandHold)
    {
      transition_to(MotionState::kStandHold);
    }
  }

  void restart_auto_sequence_walk_stage()
  {
    state_elapsed_ = 0.0;
    if (reset_phase_on_walk_start_)
    {
      global_phase_ = walk_start_phase_;
    }
  }

  AutoSequenceStage active_gait_stage() const
  {
    return action_command_active_ ? action_command_stage_ : auto_sequence_stage_;
  }

  bool gait_stage_control_active() const
  {
    return action_command_active_ || enable_auto_sequence_;
  }

  static bool gait_action_requests_walk(AutoSequenceStage stage)
  {
    return stage == AutoSequenceStage::kInPlaceStep ||
      stage == AutoSequenceStage::kForward ||
      stage == AutoSequenceStage::kBackward ||
      stage == AutoSequenceStage::kTurnRight ||
      stage == AutoSequenceStage::kTurnLeft;
  }

  static std::string normalize_action_name(std::string action)
  {
    action = to_lower_copy(action);
    for (char & c : action)
    {
      if (c == '-' || c == ' ' || c == '/')
      {
        c = '_';
      }
    }
    const auto first = action.find_first_not_of("_\t\n\r");
    if (first == std::string::npos)
    {
      return "";
    }
    const auto last = action.find_last_not_of("_\t\n\r");
    return action.substr(first, last - first + 1);
  }

  static bool parse_gait_action(const std::string & action, AutoSequenceStage & stage)
  {
    if (action == "stand" || action == "stop" || action == "idle" || action == "hold")
    {
      stage = AutoSequenceStage::kFinalStand;
      return true;
    }
    if (action == "step" || action == "in_place" || action == "in_place_step" ||
      action == "march")
    {
      stage = AutoSequenceStage::kInPlaceStep;
      return true;
    }
    if (action == "forward" || action == "fwd")
    {
      stage = AutoSequenceStage::kForward;
      return true;
    }
    if (action == "backward" || action == "back" || action == "reverse")
    {
      stage = AutoSequenceStage::kBackward;
      return true;
    }
    if (action == "turn_right" || action == "right")
    {
      stage = AutoSequenceStage::kTurnRight;
      return true;
    }
    if (action == "turn_left" || action == "left")
    {
      stage = AutoSequenceStage::kTurnLeft;
      return true;
    }
    return false;
  }

  // 启动站立早期降低支撑/前馈权重，避免刚起身时力矩过猛。
  double current_support_scale() const
  {
    if (motion_state_ != MotionState::kStandUp || stand_up_duration_ <= 1e-6)
    {
      return 1.0;
    }

    return interpolate(startup_min_support_scale_, 1.0, current_stand_alpha());
  }

  // 进入 walk 态后，不是瞬间使用完整步态目标，而是按 walk_ramp_duration_
  // 缓慢把 gait 轨迹混进来。
  double current_gait_blend() const
  {
    if (motion_state_ != MotionState::kWalk)
    {
      return 0.0;
    }

    if (walk_ramp_duration_ <= 1e-6)
    {
      return 1.0;
    }

    return smoothstep(state_elapsed_ / walk_ramp_duration_);
  }

  bool auto_sequence_motion_active() const
  {
    const AutoSequenceStage stage = active_gait_stage();
    return gait_stage_control_active() &&
      (stage == AutoSequenceStage::kForward ||
      stage == AutoSequenceStage::kBackward ||
      stage == AutoSequenceStage::kTurnRight ||
      stage == AutoSequenceStage::kTurnLeft);
  }

  double current_auto_sequence_motion_alpha() const
  {
    if (!auto_sequence_motion_active())
    {
      return 0.0;
    }

    if (walk_ramp_duration_ <= 1e-6)
    {
      return 1.0;
    }

    return smoothstep(state_elapsed_ / walk_ramp_duration_);
  }

  double current_auto_sequence_step_length() const
  {
    switch (active_gait_stage())
    {
      case AutoSequenceStage::kForward:
        return auto_sequence_forward_step_length_;
      case AutoSequenceStage::kBackward:
        return auto_sequence_backward_step_length_;
      case AutoSequenceStage::kTurnRight:
        return auto_sequence_turn_step_length_;
      case AutoSequenceStage::kTurnLeft:
        return auto_sequence_turn_left_step_length_;
      case AutoSequenceStage::kInactive:
      case AutoSequenceStage::kInitialStand:
      case AutoSequenceStage::kInPlaceStep:
      case AutoSequenceStage::kFinalStand:
        return step_length_;
    }

    return step_length_;
  }

  double current_auto_sequence_stride_direction(size_t leg_index) const
  {
    const bool left_side_leg = leg_index == 0 || leg_index == 1;
    const double command_sign = current_auto_sequence_step_length() >= 0.0 ? 1.0 : -1.0;
    switch (active_gait_stage())
    {
      // Fast real-robot demo remap from observed behavior:
      // - old same-side command produced turning
      // - old split-side command produced forward/backward
      // Keep stage names semantic, but swap the command shape here.
      case AutoSequenceStage::kForward:
      case AutoSequenceStage::kBackward:
        return left_side_leg ? -command_sign : command_sign;
      case AutoSequenceStage::kTurnRight:
      case AutoSequenceStage::kTurnLeft:
        return -command_sign;
      case AutoSequenceStage::kInactive:
      case AutoSequenceStage::kInitialStand:
      case AutoSequenceStage::kInPlaceStep:
      case AutoSequenceStage::kFinalStand:
        return 1.0;
    }

    return 1.0;
  }

  double current_auto_sequence_hfe_amplitude(size_t leg_index) const
  {
    switch (active_gait_stage())
    {
      case AutoSequenceStage::kForward:
        return auto_sequence_forward_hfe_amplitude_ *
          auto_sequence_forward_hfe_scales_[leg_index];
      case AutoSequenceStage::kBackward:
        return auto_sequence_backward_hfe_amplitude_ *
          auto_sequence_backward_hfe_scales_[leg_index];
      case AutoSequenceStage::kTurnRight:
      case AutoSequenceStage::kTurnLeft:
        return auto_sequence_turn_hfe_amplitude_ *
          auto_sequence_turn_hfe_scales_[leg_index];
      case AutoSequenceStage::kInactive:
      case AutoSequenceStage::kInitialStand:
      case AutoSequenceStage::kInPlaceStep:
      case AutoSequenceStage::kFinalStand:
        return 0.0;
    }

    return 0.0;
  }

  // 启动阶段允许用更软的增益站起来，减少冲击和震荡。
  double current_gain_scale() const
  {
    if (motion_state_ == MotionState::kWalk)
    {
      return walk_gain_scale_;
    }

    if (motion_state_ != MotionState::kStandUp || stand_up_duration_ <= 1e-6)
    {
      return stand_gain_scale_;
    }

    return interpolate(startup_initial_gain_scale_, stand_gain_scale_, current_stand_alpha());
  }

  // stand_alpha 表示整个站起过程的总进度。
  double current_stand_alpha() const
  {
    return motion_state_ == MotionState::kStandUp ?
      smoothstep(current_stand_phase()) :
      1.0;
  }

  // stand_phase 只在 kStandUp 阶段从 0 演化到 1，其余阶段直接视为完成。
  double current_stand_phase() const
  {
    if (motion_state_ != MotionState::kStandUp)
    {
      return 1.0;
    }

    if (!startup_pose_initialized())
    {
      return 0.0;
    }

    if (state_elapsed_ <= startup_settle_duration_)
    {
      return 0.0;
    }

    if (stand_up_duration_ <= 1e-6)
    {
      return 1.0;
    }

    return std::clamp((state_elapsed_ - startup_settle_duration_) / stand_up_duration_, 0.0, 1.0);
  }

  // “先动膝、后动髋”的站起策略通过两个独立 alpha 实现:
  // current_stand_knee_alpha() 更早开始增长。
  double current_stand_knee_alpha() const
  {
    if (motion_state_ != MotionState::kStandUp)
    {
      return 1.0;
    }

    if (startup_knee_first_ratio_ <= 1e-6)
    {
      return current_stand_alpha();
    }

    return smoothstep(current_stand_phase() / startup_knee_first_ratio_);
  }

  // 髋关节 alpha 相对膝关节延后，形成更稳定的站起动作。
  double current_stand_hip_alpha() const
  {
    if (motion_state_ != MotionState::kStandUp)
    {
      return 1.0;
    }

    if (startup_knee_first_ratio_ >= 1.0 - 1e-6)
    {
      return 0.0;
    }

    const double delayed_phase =
      (current_stand_phase() - startup_knee_first_ratio_) / (1.0 - startup_knee_first_ratio_);
    return smoothstep(delayed_phase);
  }

  double startup_phase_window_alpha(double phase, double start, double end) const
  {
    if (motion_state_ != MotionState::kStandUp)
    {
      return 1.0;
    }

    if (end <= start + 1e-6)
    {
      return phase >= end ? 1.0 : 0.0;
    }

    return smoothstep((phase - start) / (end - start));
  }

  double direct_step_lift_direction(
    size_t leg_index,
    const LegState & leg,
    double hfe_command,
    double kfe_command,
    size_t joint_index) const
  {
    constexpr double kProbeDelta = 0.02;
    const auto foot_y_with_delta =
      [&](double hfe_delta, double kfe_delta) {
        const LegPose2D pose = forward_kinematics(
          geometries_[leg_index],
          hfe_command + hfe_delta - leg.hfe_offset,
          kfe_command + kfe_delta - leg.kfe_offset);
        return pose.y;
      };

    const double plus_y = joint_index == kHfeJointIndex ?
      foot_y_with_delta(kProbeDelta, 0.0) :
      foot_y_with_delta(0.0, kProbeDelta);
    const double minus_y = joint_index == kHfeJointIndex ?
      foot_y_with_delta(-kProbeDelta, 0.0) :
      foot_y_with_delta(0.0, -kProbeDelta);

    return plus_y < minus_y ? 1.0 : -1.0;
  }

  double direct_step_forward_direction(
    size_t leg_index,
    const LegState & leg,
    double hfe_command,
    double kfe_command) const
  {
    constexpr double kProbeDelta = 0.02;
    const auto foot_x_with_delta =
      [&](double hfe_delta) {
        const LegPose2D pose = forward_kinematics(
          geometries_[leg_index],
          hfe_command + hfe_delta - leg.hfe_offset,
          kfe_command - leg.kfe_offset);
        return pose.x;
      };

    const double plus_x = foot_x_with_delta(kProbeDelta);
    const double minus_x = foot_x_with_delta(-kProbeDelta);
    return plus_x > minus_x ? 1.0 : -1.0;
  }

  // 一旦收到完整 joint_states，就把该时刻的真实关节角记为站立插值起点。
  // 这能确保“从当前姿态站起”，而不是从一组假定初值突兀跳转。
  void maybe_capture_stand_start_positions()
  {
    if (stand_start_positions_captured_ || !joint_state_received_)
    {
      return;
    }

    const size_t valid_joint_count = std::count(
      actual_position_valid_.begin(), actual_position_valid_.end(), true);
    if (valid_joint_count == 0)
    {
      return;
    }

    std::vector<std::string> missing_joint_names;
    if (valid_joint_count != kTotalJointCount)
    {
      if (!allow_partial_joint_state_startup_)
      {
        return;
      }

      missing_joint_names.reserve(kTotalJointCount - valid_joint_count);
      for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
      {
        for (size_t joint_index = 0; joint_index < kJointsPerLeg; ++joint_index)
        {
          const size_t flat_index = flat_joint_index(leg_index, joint_index);
          if (!actual_position_valid_[flat_index])
          {
            missing_joint_names.push_back(joint_name(leg_index, joint_index));
          }
        }
      }
    }

    for (size_t flat_index = 0; flat_index < kTotalJointCount; ++flat_index)
    {
      const double startup_position = actual_position_valid_[flat_index] ?
        actual_positions_[flat_index] :
        commanded_positions_[flat_index];
      stand_start_positions_[flat_index] = startup_position;
      slewed_command_positions_[flat_index] = startup_position;
      slewed_command_initialized_[flat_index] = true;
    }

    if (use_startup_crouch_pose_ && use_direct_joint_stand_targets_ && !joint_isolation_kfe_only())
    {
      for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
      {
        set_startup_crouch_targets(leg_index, false);
      }
    }

    stand_start_positions_captured_ = true;
    if (!missing_joint_names.empty())
    {
      std::ostringstream oss;
      for (size_t i = 0; i < missing_joint_names.size(); ++i)
      {
        if (i != 0)
        {
          oss << ", ";
        }
        oss << missing_joint_names[i];
      }

      RCLCPP_WARN(
        get_logger(),
        "Startup continuing with partial joint states: valid=%zu/%zu missing=[%s]",
        valid_joint_count, kTotalJointCount, oss.str().c_str());
    }
    else
    {
      RCLCPP_INFO(
        get_logger(), "Captured complete startup joint state: %zu/%zu",
        valid_joint_count, kTotalJointCount);
    }
  }

  // 启动姿态是否已经具备:
  // - 要么已经抓到了真实的起始关节位置
  // - 要么使用了预置的 crouch 姿态
  bool startup_pose_initialized() const
  {
    return stand_start_positions_captured_ ||
      (use_startup_crouch_pose_ && startup_crouch_initialized_ && !joint_isolation_kfe_only());
  }

  // 当前支持的简易关节隔离模式: 只测试 KFE。
  bool joint_isolation_kfe_only() const
  {
    return joint_isolation_mode_ == "kfe_only";
  }

  // 建立 joint name -> flat index 的查找表，避免回调里反复字符串比较。
  void initialize_joint_name_to_index()
  {
    for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
      for (size_t joint_index = 0; joint_index < kJointsPerLeg; ++joint_index)
      {
        joint_name_to_index_.emplace(
          joint_name(leg_index, joint_index),
          flat_joint_index(leg_index, joint_index));
      }
    }
  }

  // 直接关节目标模式: 不通过 IK，而是直接使用配置里的关节角目标。
  // 常用于启动站立、低趴姿态、隔离测试等场景。
  JointTargets direct_targets_for_leg(
    size_t leg_index,
    const std::vector<double> & haa_targets,
    const std::vector<double> & hfe_targets,
    const std::vector<double> & kfe_targets) const
  {
    return {
      direct_target_value(haa_targets, direct_haa_signs_, leg_index, kHaaLower, kHaaUpper),
      direct_target_value(hfe_targets, direct_hfe_signs_, leg_index, kHfeLower, kHfeUpper),
      direct_target_value(kfe_targets, direct_kfe_signs_, leg_index, kKfeLower, kKfeUpper)};
  }

  // 统一生成“站姿目标”。
  // 如果 use_direct_joint_stand_targets_ 为真，则直接读取配置关节角；
  // 否则基于站姿足端位置做 IK，再叠加零位偏置。
  JointTargets stand_targets_for_leg(
    size_t leg_index,
    const LegState & leg,
    bool hold_startup_crouch_target) const
  {
    if (use_direct_joint_stand_targets_)
    {
      if (hold_startup_crouch_target)
      {
        return direct_targets_for_leg(
          leg_index,
          startup_crouch_haa_targets_,
          startup_crouch_hfe_targets_,
          startup_crouch_kfe_targets_);
      }

      return direct_targets_for_leg(
        leg_index,
        stand_haa_targets_,
        stand_hfe_targets_,
        stand_kfe_targets_);
    }

    const double stand_foot_x = foot_x_signs_[leg_index] * stand_foot_x_offsets_[leg_index];
    const IKResult stand_ik = inverse_kinematics(
      geometries_[leg_index], stand_foot_x, stance_depth_);
    return {
      std::clamp(nominal_haa_angles_[leg_index], kHaaLower, kHaaUpper),
      std::clamp(stand_ik.hfe + leg.hfe_offset, kHfeLower, kHfeUpper),
      std::clamp(stand_ik.kfe + leg.kfe_offset, kKfeLower, kKfeUpper)};
  }

  // 当前站立插值起点优先使用真实采样值；若尚未采到，则退化到上一次命令值。
  double current_stand_start_position(size_t flat_index) const
  {
    if (stand_start_positions_captured_)
    {
      return stand_start_positions_[flat_index];
    }

    return actual_position_valid_[flat_index] ?
      actual_positions_[flat_index] :
      commanded_positions_[flat_index];
  }

  // 读取一条腿三个关节当前的“站立起点”。
  JointTargets current_stand_start_targets(const JointIndices & indices) const
  {
    return {
      current_stand_start_position(indices.haa),
      current_stand_start_position(indices.hfe),
      current_stand_start_position(indices.kfe)};
  }

  // 预置 crouch 目标到多个内部缓存数组中，保证后续插值和 slew limit
  // 都以一致的起点开始。
  void set_startup_crouch_targets(size_t leg_index, bool initialize_commanded_positions)
  {
    const auto indices = joint_indices(leg_index);
    const JointTargets targets = direct_targets_for_leg(
      leg_index,
      startup_crouch_haa_targets_,
      startup_crouch_hfe_targets_,
      startup_crouch_kfe_targets_);

    set_leg_joint_values(stand_start_positions_, indices, targets);
    set_leg_joint_values(slewed_command_positions_, indices, targets);
    slewed_command_initialized_[indices.haa] = true;
    slewed_command_initialized_[indices.hfe] = true;
    slewed_command_initialized_[indices.kfe] = true;

    if (initialize_commanded_positions)
    {
      set_leg_joint_values(commanded_positions_, indices, targets);
    }
  }

  // 对四条腿统一应用 crouch 预置。
  void initialize_startup_crouch_positions()
  {
    for (size_t leg_index = 0; leg_index < kLegCount; ++leg_index)
    {
      set_startup_crouch_targets(leg_index, true);
    }

    startup_crouch_initialized_ = true;
  }

  // 站立阶段的命令变化率限制器。
  // 目的不是降低跟踪误差，而是控制“命令本身”的平滑性，避免启动瞬间
  // 给 mit_controller 一个过陡的参考阶跃。
  double apply_command_slew_limit(
    size_t flat_index,
    size_t joint_index,
    double target,
    double dt)
  {
    if (!slewed_command_initialized_[flat_index])
    {
      slewed_command_positions_[flat_index] =
        actual_position_valid_[flat_index] ? actual_positions_[flat_index] : target;
      slewed_command_initialized_[flat_index] = true;
    }

    if (motion_state_ == MotionState::kWalk)
    {
      slewed_command_positions_[flat_index] = target;
      return target;
    }

    double slew_rate = stand_kfe_slew_rate_;
    if (joint_index == kHaaJointIndex)
    {
      slew_rate = stand_haa_slew_rate_;
    }
    else if (joint_index == kHfeJointIndex)
    {
      slew_rate = stand_hfe_slew_rate_;
    }

    slewed_command_positions_[flat_index] = slew_limit(
      slewed_command_positions_[flat_index],
      target,
      slew_rate * std::max(dt, 0.0));
    return slewed_command_positions_[flat_index];
  }

  // 大腿关节的重力前馈近似:
  // 把等效大腿质量与小腿质量都折算到 HFE 关节上。
  double compute_thigh_feedforward(size_t leg_index, double hfe, double kfe) const
  {
    const auto & geometry = geometries_[leg_index];
    const auto & leg = leg_states_[leg_index];

    const double thigh_term =
      leg.thigh_mass * 9.81 * (geometry.thigh_length * 0.5) * std::cos(hfe);
    const double shank_term =
      leg.shank_mass * 9.81 *
      (geometry.thigh_length * std::cos(hfe) +
      geometry.shank_length * 0.5 * std::cos(hfe + kfe));

    return thigh_term + shank_term;
  }

  // 膝关节重力前馈近似:
  // 这里只考虑小腿等效质量对 KFE 的贡献。
  double compute_knee_feedforward(size_t leg_index, double hfe, double kfe) const
  {
    const auto & geometry = geometries_[leg_index];
    const auto & leg = leg_states_[leg_index];

    return -leg.shank_mass * 9.81 * (geometry.shank_length * 0.5) * std::cos(hfe + kfe);
  }

  // 切换运动状态，并重置该状态下已经消耗的时间。
  void transition_to(MotionState next_state)
  {
    if (motion_state_ == next_state)
    {
      return;
    }

    motion_state_ = next_state;
    state_elapsed_ = 0.0;
    if (motion_state_ == MotionState::kWalk && reset_phase_on_walk_start_)
    {
      global_phase_ = walk_start_phase_;
    }
    RCLCPP_INFO(get_logger(), "Motion state -> %s", motion_state_name(motion_state_));
  }

  void transition_auto_sequence_stage(AutoSequenceStage next_stage)
  {
    if (auto_sequence_stage_ == next_stage)
    {
      return;
    }

    auto_sequence_stage_ = next_stage;
    auto_sequence_elapsed_ = 0.0;
    RCLCPP_INFO(
      get_logger(), "Auto sequence stage -> %s",
      auto_sequence_stage_name(auto_sequence_stage_));
  }

  // 仅用于日志输出的人类可读状态名。
  static const char * motion_state_name(MotionState state)
  {
    switch (state)
    {
      case MotionState::kStandUp:
        return "stand_up";
      case MotionState::kStandHold:
        return "stand_hold";
      case MotionState::kWalk:
        return "walk";
    }

    return "unknown";
  }

  static const char * auto_sequence_stage_name(AutoSequenceStage stage)
  {
    switch (stage)
    {
      case AutoSequenceStage::kInactive:
        return "inactive";
      case AutoSequenceStage::kInitialStand:
        return "initial_stand";
      case AutoSequenceStage::kInPlaceStep:
        return "in_place_step";
      case AutoSequenceStage::kForward:
        return "forward";
      case AutoSequenceStage::kBackward:
        return "backward";
      case AutoSequenceStage::kTurnRight:
        return "turn_right";
      case AutoSequenceStage::kTurnLeft:
        return "turn_left";
      case AutoSequenceStage::kFinalStand:
        return "final_stand";
    }

    return "unknown";
  }

  static const char * gait_action_stage_name(AutoSequenceStage stage)
  {
    switch (stage)
    {
      case AutoSequenceStage::kInPlaceStep:
        return "step";
      case AutoSequenceStage::kForward:
        return "forward";
      case AutoSequenceStage::kBackward:
        return "backward";
      case AutoSequenceStage::kTurnRight:
        return "turn_right";
      case AutoSequenceStage::kTurnLeft:
        return "turn_left";
      case AutoSequenceStage::kInactive:
      case AutoSequenceStage::kInitialStand:
      case AutoSequenceStage::kFinalStand:
        return "stand";
    }

    return "unknown";
  }

  // 成员变量按用途分组:
  // - 标量参数
  // - 按腿数组参数
  // - 增益
  // - 运行期状态缓存
  // - ROS 通信对象
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string gait_action_topic_;
  std::string gait_profile_{"legacy_walk"};
  double step_length_{0.10};
  double step_height_{0.05};
  double gait_period_{3.0};
  double stance_depth_{0.33};
  double control_rate_{50.0};
  double swing_ratio_{0.7};
  double stance_transition_ratio_{0.15};
  bool reset_phase_on_walk_start_{true};
  double walk_start_phase_{0.0};
  double swing_gain_scale_{0.5};
  double swing_hfe_motion_scale_{0.45};
  double swing_extra_kfe_flexion_{0.18};
  bool use_direct_joint_in_place_step_{false};
  double direct_step_hfe_amplitude_{0.0};
  double direct_step_kfe_amplitude_{0.0};
  double direct_step_support_hfe_amplitude_{0.0};
  double direct_step_support_kfe_amplitude_{0.0};
  std::vector<double> direct_step_hfe_signs_;
  std::vector<double> direct_step_kfe_signs_;
  std::vector<double> direct_step_hfe_swing_scales_;
  std::vector<double> direct_step_kfe_swing_scales_;
  std::vector<double> direct_step_hfe_support_scales_;
  std::vector<double> direct_step_kfe_support_scales_;
  double rear_swing_step_length_scale_{1.0};
  double rear_swing_step_height_scale_{1.0};
  double rear_swing_gain_scale_{1.0};
  double rear_swing_hfe_motion_scale_{1.0};
  double rear_swing_extra_kfe_flexion_scale_{1.0};
  double rear_swing_pitch_feedforward_offset_{0.0};
  double rear_swing_body_x_shift_{0.0};
  double support_pitch_balance_gain_{0.0};
  double support_pitch_balance_max_offset_{0.0};
  double support_pitch_balance_target_delta_{0.0};
  double swing_period_{2.1};
  double stance_period_{0.9};
  bool stand_only_{true};
  bool enable_startup_stand_{true};
  double stand_up_duration_{3.0};
  double startup_knee_first_ratio_{0.60};
  bool startup_front_legs_first_{false};
  double startup_front_leg_complete_ratio_{1.0};
  double startup_rear_hip_start_ratio_{0.0};
  double startup_rear_knee_start_ratio_{0.0};
  double stand_hold_duration_{1.0};
  double walk_ramp_duration_{1.0};
  double stand_gain_scale_{1.8};
  double walk_gain_scale_{1.0};
  bool enable_auto_sequence_{false};
  double auto_sequence_initial_stand_duration_{2.0};
  double auto_sequence_in_place_step_duration_{4.0};
  double auto_sequence_forward_duration_{5.0};
  double auto_sequence_backward_duration_{5.0};
  double auto_sequence_turn_right_duration_{5.0};
  double auto_sequence_turn_left_duration_{5.0};
  bool auto_sequence_forward_use_direct_step_{true};
  double auto_sequence_forward_step_length_{0.02};
  double auto_sequence_backward_step_length_{-0.02};
  double auto_sequence_turn_step_length_{0.015};
  double auto_sequence_turn_left_step_length_{-0.015};
  double auto_sequence_forward_step_height_{0.012};
  double auto_sequence_forward_hfe_amplitude_{0.02};
  double auto_sequence_backward_hfe_amplitude_{0.02};
  double auto_sequence_turn_hfe_amplitude_{0.018};
  double stand_haa_slew_rate_{0.15};
  double stand_hfe_slew_rate_{0.20};
  double stand_kfe_slew_rate_{0.25};
  double gravity_ff_scale_{2.2};
  double shank_mass_ratio_{0.5};
  bool debug_joint_tracking_{false};
  double debug_log_period_{0.5};
  bool auto_zero_offsets_{true};
  bool use_direct_joint_stand_targets_{true};
  bool use_startup_crouch_pose_{true};
  bool allow_partial_joint_state_startup_{false};
  std::string joint_isolation_mode_{"off"};
  double joint_isolation_kfe_target_{-0.70};
  double startup_settle_duration_{2.0};
  double startup_min_support_scale_{0.05};
  double startup_initial_gain_scale_{0.15};
  double global_phase_{0.0};
  double state_elapsed_{0.0};
  MotionState motion_state_{MotionState::kStandUp};
  AutoSequenceStage auto_sequence_stage_{AutoSequenceStage::kInactive};
  AutoSequenceStage action_command_stage_{AutoSequenceStage::kFinalStand};
  double auto_sequence_elapsed_{0.0};
  bool action_command_active_{false};

  std::vector<double> phase_offsets_;
  std::vector<double> foot_x_signs_;
  std::vector<double> gait_foot_x_offsets_;
  std::vector<double> stand_foot_x_offsets_;
  std::vector<double> nominal_haa_angles_;
  std::vector<double> nominal_hfe_angles_;
  std::vector<double> nominal_kfe_angles_;
  std::vector<double> direct_haa_signs_;
  std::vector<double> direct_hfe_signs_;
  std::vector<double> direct_kfe_signs_;
  std::vector<double> direct_step_swing_scales_;
  std::vector<double> direct_step_support_scales_;
  std::vector<double> auto_sequence_forward_hfe_scales_;
  std::vector<double> auto_sequence_backward_hfe_scales_;
  std::vector<double> auto_sequence_turn_hfe_scales_;
  std::vector<double> stand_haa_targets_;
  std::vector<double> stand_hfe_targets_;
  std::vector<double> stand_kfe_targets_;
  std::vector<double> startup_crouch_haa_targets_;
  std::vector<double> startup_crouch_hfe_targets_;
  std::vector<double> startup_crouch_kfe_targets_;
  std::vector<double> manual_hfe_offsets_;
  std::vector<double> manual_kfe_offsets_;

  double haa_kp_{20.0};
  double haa_kd_{2.2};
  double hfe_kp_{70.0};
  double hfe_kd_{7.5};
  double kfe_kp_{60.0};
  double kfe_kd_{6.0};

  std::array<LegGeometry, kLegCount> geometries_{};
  std::array<LegState, kLegCount> leg_states_{};
  std::array<double, kTotalJointCount> commanded_positions_{};
  std::array<double, kTotalJointCount> actual_positions_{};
  std::array<double, kTotalJointCount> stand_start_positions_{};
  std::array<double, kTotalJointCount> slewed_command_positions_{};
  std::array<bool, kTotalJointCount> actual_position_valid_{};
  std::array<bool, kTotalJointCount> slewed_command_initialized_{};
  std::unordered_map<std::string, size_t> joint_name_to_index_;
  bool joint_state_received_{false};
  bool stand_start_positions_captured_{false};
  bool startup_crouch_initialized_{false};

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_log_time_{0, 0, RCL_ROS_TIME};
  bool last_time_initialized_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_action_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // cmd_vel 遥控状态
  double cmd_vel_linear_x_{0.0};
  double cmd_vel_angular_z_{0.0};
  double cmd_vel_timeout_{0.5};
  double max_linear_vel_{0.5};
  double max_angular_vel_{1.0};
  double cmd_vel_deadzone_{0.05};
  bool cmd_vel_received_{false};
  bool teleop_walk_requested_{false};
  bool startup_complete_{false};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace dog_position_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dog_position_control::QuadrupedGaitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
