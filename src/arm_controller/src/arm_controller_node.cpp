#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "arm_controller/kinematics_types.hpp"

namespace arm_controller
{

namespace
{

/// Cubic smoothstep: t²(3-2t) for t in [0,1], with zero derivative at endpoints.
double smoothstep(const double t)
{
  const double clamped = std::max(0.0, std::min(t, 1.0));
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

}  // namespace

/**
 * @brief 机械臂上层控制节点
 *
 * 该节点负责把“目标空间点”转换成底层电机能执行的 MIT 控制命令，整体流程为：
 * 1. 订阅当前关节状态，维护逆运动学初始种子和轨迹起点。
 * 2. 订阅手动目标点 /arm_target_point，调用位置 IK 得到目标关节角。
 * 3. 订阅箱子动作触发话题，生成接近、接触、提升和回收等任务关键点。
 * 4. 订阅回原位触发话题，先关闭吸盘，再回到配置的 home 关节角。
 * 5. 将 IK 或 home 目标在关节空间内做 smoothstep 平滑插值。
 * 6. 周期发布 [p, v, kp, kd, t_ff] * 4 格式的 MIT 命令。
 *
 * 节点本身不直接操作硬件。仿真或真机后端会订阅 MIT 命令话题，并转换为 ros2_control
 * 位置命令、力矩命令或真实 DM 电机 CAN 帧。
 */
class ArmControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造 ROS 2 控制节点并建立全部通信接口
   *
   * 初始化顺序：
   * 1. 声明参数，提供可运行的默认配置。
   * 2. 读取参数并进行长度、范围和派生值校验。
   * 3. 创建目标点、箱子动作触发、回原位触发、关节状态订阅器。
   * 4. 创建 MIT 电机命令和吸盘开关发布器。
   * 5. 创建固定频率定时器，周期执行轨迹插值与命令发布。
   */
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    declare_parameters();
    load_parameters();

    target_subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_point_topic_,
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        handle_target_point(*msg);
      });

    box_motion_start_subscriber_ = create_subscription<std_msgs::msg::Empty>(
      box_motion_start_topic_,
      10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        handle_box_motion_start(*msg);
      });

    home_motion_start_subscriber_ = create_subscription<std_msgs::msg::Empty>(
      home_motion_start_topic_,
      10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        handle_home_motion_start(*msg);
      });

    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        handle_joint_state(*msg);
      });

    mit_command_publisher_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(mit_command_topic_, 10);
    suction_command_publisher_ =
      create_publisher<std_msgs::msg::Bool>(suction_command_topic_, 10);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
    command_timer_ = create_wall_timer(timer_period, [this]() {
        publish_command_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "arm_controller is ready: target=%s, box_start=%s, home_start=%s, joint_states=%s, mit_commands=%s",
      target_point_topic_.c_str(),
      box_motion_start_topic_.c_str(),
      home_motion_start_topic_.c_str(),
      joint_states_topic_.c_str(),
      mit_command_topic_.c_str());
  }

private:
  /**
   * @brief 声明节点运行所需的全部 ROS 参数
   *
   * 参数分为四类：
   * 1. 话题与坐标系参数：决定节点订阅和发布的接口名称。
   * 2. 机械臂几何参数：关节原点、关节轴、关节限位和工具偏移，用于 FK/IK。
   * 3. 轨迹和 MIT 控制参数：控制插值速度、指令频率、运动/保持阶段的 Kp/Kd。
   * 4. 箱子任务参数：描述箱子尺寸、目标吸附位置、接近距离、提升高度和姿态约束。
   *
   * 这些默认值会被 launch 或 YAML 配置覆盖。
   */
  void declare_parameters()
  {
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "base_yaw_joint",
        "shoulder_pitch_joint",
        "elbow_pitch_joint",
        "wrist_pitch_joint",
      });
    declare_parameter<std::string>("target_point_topic", "/arm_target_point");
    declare_parameter<std::string>("box_motion_start_topic", "/arm_box_motion/start");
    declare_parameter<std::string>("home_motion_start_topic", "/arm_home_motion/start");
    declare_parameter<std::string>("suction_command_topic", "/suction/enable");
    declare_parameter<std::string>("joint_states_topic", "/dm_j4340/joint_states");
    declare_parameter<std::string>("mit_command_topic", "/dm_j4340/mit_commands");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<std::vector<double>>(
      "joint_origins",
      {
        0.000016999, -0.000022768, 0.058,
        0.0080913, 0.029425, 0.0395,
        -0.21008, -0.0094917, 0.0,
        0.21061, 0.01142, 0.057,
      });
    declare_parameter<std::vector<double>>(
      "joint_axes",
      {
        0.0, 0.0, 1.0,
        0.26585, 0.96402, 0.0,
        -0.26585, -0.96402, 0.0,
        0.26585, 0.96402, 0.0,
      });
    declare_parameter<std::vector<double>>("joint_lower_limits", {-3.14, -3.14, -3.14, -3.14});
    declare_parameter<std::vector<double>>("joint_upper_limits", {3.14, 3.14, 3.14, 3.14});
    declare_parameter<std::vector<double>>("tool_offset", {0.16309, -0.080763, -0.016359});

    declare_parameter<double>("position_tolerance_m", 0.005);
    declare_parameter<int>("max_iterations", 100);
    declare_parameter<double>("damping", 0.04);
    declare_parameter<double>("max_solver_step_rad", 0.12);
    declare_parameter<double>("target_repeat_deadband_m", 0.002);

    declare_parameter<double>("command_rate_hz", 50.0);
    declare_parameter<double>("max_joint_speed_rad_s", 0.18);
    declare_parameter<double>("min_trajectory_duration_s", 1.2);
    declare_parameter<bool>("hold_last_command", true);
    declare_parameter<std::vector<double>>("kp", {5.0, 8.0, 7.0, 3.0});
    declare_parameter<std::vector<double>>("kd", {2.6, 4.0, 3.5, 1.8});
    declare_parameter<std::vector<double>>("hold_kp", {2.0, 3.0, 2.5, 1.2});
    declare_parameter<std::vector<double>>("hold_kd", {3.0, 4.5, 4.0, 2.0});
    declare_parameter<std::vector<double>>("command_velocity", {0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("torque_ff", {0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("home_joint_positions", {0.0, 0.0, 0.0, 0.0});

    declare_parameter<double>("box_center_distance_m", 0.75);
    declare_parameter<double>("arm_to_box_distance_m", 0.75);
    declare_parameter<double>("platform_height_m", 0.16);
    declare_parameter<double>("box_ground_z_m", 0.0);
    declare_parameter<double>("box_width_m", 0.25);
    declare_parameter<double>("box_depth_m", 0.25);
    declare_parameter<double>("box_height_m", 0.25);
    declare_parameter<double>("box_mass_kg", 0.5);
    declare_parameter<double>("box_motion_y_m", 0.0);
    declare_parameter<double>("box_motion_base_yaw_limit_rad", 0.50);
    declare_parameter<double>("box_motion_tool_direction_tolerance", 0.35);
    declare_parameter<double>("box_motion_tool_direction_weight", 0.12);
    declare_parameter<double>("box_approach_offset_m", 0.06);
    declare_parameter<double>("box_min_approach_x_m", 0.14);
    declare_parameter<double>("box_touch_push_m", 0.005);
    declare_parameter<double>("box_touch_height_ratio", 0.5);
    declare_parameter<double>("box_min_touch_z_m", -0.06);
    declare_parameter<double>("box_suction_target_x_m", 0.0);
    declare_parameter<double>("box_suction_approach_clearance_m", 0.06);
    declare_parameter<double>("box_suction_contact_offset_m", 0.01);
    declare_parameter<double>("box_suction_settle_s", 0.8);
    declare_parameter<double>("box_lift_height_m", 0.08);
    declare_parameter<double>("box_lift_retract_m", 0.04);
    declare_parameter<double>("box_motion_step_wait_s", 0.5);
    declare_parameter<double>("box_motion_position_tolerance_m", 0.02);
    declare_parameter<double>("box_motion_shoulder_bias_rad", 0.85);
    declare_parameter<double>("box_motion_elbow_bias_rad", 0.4);
    declare_parameter<double>("box_motion_wrist_bias_rad", -0.6);
  }

  /**
   * @brief 从 ROS 参数服务器读取配置并写入成员变量
   *
   * 读取过程中会对关键参数进行基础校验和安全处理：
   * - 数组参数必须满足固定长度，例如 joint_origins 为 12 个 double。
   * - 频率、速度、尺寸、迭代次数等参数会被限制为正值或非负值。
   * - 姿态容差、箱子触点高度比例、箱子动作 yaw 范围会被限制在合理区间。
   * - 未显式设置吸盘目标 x 坐标时，自动使用箱子朝向机械臂那一面的中心。
   * - 当前关节初值初始化为每个关节软件限位的中点，避免尚未收到 JointState 时从零值乱跳。
   *
   * @throws std::runtime_error 当数组长度不匹配或关节下限大于上限时抛出异常
   */
  void load_parameters()
  {
    target_point_topic_ = get_parameter("target_point_topic").as_string();
    box_motion_start_topic_ = get_parameter("box_motion_start_topic").as_string();
    home_motion_start_topic_ = get_parameter("home_motion_start_topic").as_string();
    suction_command_topic_ = get_parameter("suction_command_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    mit_command_topic_ = get_parameter("mit_command_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    const std::vector<std::string> joint_names =
      get_parameter("joint_names").as_string_array();
    if (joint_names.size() != kJointCount) {
      throw std::runtime_error("joint_names must contain exactly 4 names.");
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      joint_names_[index] = joint_names[index];
    }

    joint_origins_ = read_vector3_array("joint_origins");
    joint_axes_ = read_vector3_array("joint_axes");
    lower_limits_ = read_double_array("joint_lower_limits");
    upper_limits_ = read_double_array("joint_upper_limits");
    tool_offset_ = read_vector3("tool_offset");
    kp_ = read_double_array("kp");
    kd_ = read_double_array("kd");
    hold_kp_ = read_double_array("hold_kp");
    hold_kd_ = read_double_array("hold_kd");
    command_velocity_ = read_double_array("command_velocity");
    torque_ff_ = read_double_array("torque_ff");
    const std::array<double, kJointCount> home_joint_positions =
      read_double_array("home_joint_positions");

    position_tolerance_m_ = positive_parameter("position_tolerance_m", 0.001);
    max_iterations_ = std::max(1, static_cast<int>(get_parameter("max_iterations").as_int()));
    damping_ = positive_parameter("damping", 0.001);
    max_solver_step_rad_ = positive_parameter("max_solver_step_rad", 0.01);
    target_repeat_deadband_m_ = std::max(0.0, get_parameter("target_repeat_deadband_m").as_double());
    command_rate_hz_ = positive_parameter("command_rate_hz", 1.0);
    max_joint_speed_rad_s_ = positive_parameter("max_joint_speed_rad_s", 0.01);
    min_trajectory_duration_s_ = std::max(0.0, get_parameter("min_trajectory_duration_s").as_double());
    hold_last_command_ = get_parameter("hold_last_command").as_bool();
    box_center_distance_m_ = positive_parameter("box_center_distance_m", 0.01);
    arm_to_box_distance_m_ = positive_parameter("arm_to_box_distance_m", 0.01);
    platform_height_m_ = std::max(0.0, get_parameter("platform_height_m").as_double());
    box_ground_z_m_ = get_parameter("box_ground_z_m").as_double();
    box_width_m_ = positive_parameter("box_width_m", 0.01);
    box_depth_m_ = positive_parameter("box_depth_m", 0.01);
    box_height_m_ = positive_parameter("box_height_m", 0.01);
    box_mass_kg_ = positive_parameter("box_mass_kg", 0.0);
    box_motion_y_m_ = get_parameter("box_motion_y_m").as_double();
    box_motion_base_yaw_limit_rad_ = std::clamp(
      positive_parameter("box_motion_base_yaw_limit_rad", 0.01),
      0.01,
      3.14);
    box_motion_tool_direction_tolerance_ = std::clamp(
      positive_parameter("box_motion_tool_direction_tolerance", 0.01),
      0.01,
      2.0);
    box_motion_tool_direction_weight_ =
      positive_parameter("box_motion_tool_direction_weight", 0.001);
    box_approach_offset_m_ = std::max(0.0, get_parameter("box_approach_offset_m").as_double());
    box_min_approach_x_m_ = positive_parameter("box_min_approach_x_m", 0.01);
    box_touch_push_m_ = std::max(0.0, get_parameter("box_touch_push_m").as_double());
    box_touch_height_ratio_ =
      std::clamp(get_parameter("box_touch_height_ratio").as_double(), 0.05, 0.95);
    box_min_touch_z_m_ = get_parameter("box_min_touch_z_m").as_double();
    const double configured_suction_target_x =
      get_parameter("box_suction_target_x_m").as_double();
    box_suction_target_x_m_ =
      configured_suction_target_x > 0.0 ?
      configured_suction_target_x :
      box_center_distance_m_ - 0.5 * box_depth_m_;
    box_suction_approach_clearance_m_ =
      std::max(0.0, get_parameter("box_suction_approach_clearance_m").as_double());
    box_suction_contact_offset_m_ =
      std::max(0.0, get_parameter("box_suction_contact_offset_m").as_double());
    box_suction_settle_s_ = std::max(0.0, get_parameter("box_suction_settle_s").as_double());
    box_lift_height_m_ = std::max(0.0, get_parameter("box_lift_height_m").as_double());
    box_lift_retract_m_ = std::max(0.0, get_parameter("box_lift_retract_m").as_double());
    box_motion_step_wait_s_ = std::max(0.0, get_parameter("box_motion_step_wait_s").as_double());
    box_motion_position_tolerance_m_ =
      positive_parameter("box_motion_position_tolerance_m", position_tolerance_m_);
    box_motion_shoulder_bias_rad_ =
      std::clamp(
      get_parameter("box_motion_shoulder_bias_rad").as_double(),
      lower_limits_[1],
      upper_limits_[1]);
    box_motion_elbow_bias_rad_ =
      std::clamp(
      get_parameter("box_motion_elbow_bias_rad").as_double(),
      lower_limits_[2],
      upper_limits_[2]);
    box_motion_wrist_bias_rad_ =
      std::clamp(
      get_parameter("box_motion_wrist_bias_rad").as_double(),
      lower_limits_[3],
      upper_limits_[3]);

    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (lower_limits_[index] > upper_limits_[index]) {
        throw std::runtime_error("joint lower limit must not be greater than upper limit.");
      }
      current_joints_(static_cast<int>(index)) =
        0.5 * (lower_limits_[index] + upper_limits_[index]);
      home_joints_(static_cast<int>(index)) =
        std::clamp(home_joint_positions[index], lower_limits_[index], upper_limits_[index]);
    }

    // 实机安全：关节软件限位超过 ±2.5 rad 时发出醒目告警。
    // 仿真调试可以使用大范围，但真机必须按实际机械限位设定。
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double span = upper_limits_[index] - lower_limits_[index];
      if (span > 5.0 || lower_limits_[index] < -2.5 || upper_limits_[index] > 2.5) {
        RCLCPP_WARN(
          get_logger(),
          "⚠️  Joint '%s' limit range is [%.2f, %.2f] (span=%.2f rad). "
          "This is intended for simulation only! "
          "For real hardware, set limits matching the actual mechanical limits in arm_controller.yaml.",
          joint_names_[index].c_str(),
          lower_limits_[index],
          upper_limits_[index],
          span);
        break;  // 只告警一次，避免刷屏
      }
    }
  }

  /**
   * @brief 读取由连续 double 数组表示的 4 个三维向量参数
   *
   * joint_origins 和 joint_axes 都采用同一种参数布局：
   * [x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3]。
   * 该函数将其转换为 Eigen::Vector3d 数组，方便运动学函数直接使用。
   *
   * @param name 参数名
   *
   * @return std::array<Eigen::Vector3d, kJointCount> 解析后的 4 个三维向量
   *
   * @throws std::runtime_error 当参数不是 12 个 double 时抛出异常
   */
  std::array<Eigen::Vector3d, kJointCount> read_vector3_array(const std::string & name) const
  {
    const std::vector<double> values = get_parameter(name).as_double_array();
    if (values.size() != 3 * kJointCount) {
      throw std::runtime_error(name + " must contain 12 double values.");
    }

    std::array<Eigen::Vector3d, kJointCount> vectors;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      vectors[index] = Eigen::Vector3d(
        values[3 * index],
        values[3 * index + 1],
        values[3 * index + 2]);
    }
    return vectors;
  }

  /**
   * @brief 读取单个三维向量参数
   *
   * 当前主要用于读取 tool_offset，即末端工具坐标系相对 wrist_link 的固定平移。
   *
   * @param name 参数名
   *
   * @return Eigen::Vector3d 解析后的三维向量
   *
   * @throws std::runtime_error 当参数不是 3 个 double 时抛出异常
   */
  Eigen::Vector3d read_vector3(const std::string & name) const
  {
    const std::vector<double> values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain 3 double values.");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  /**
   * @brief 读取长度等于关节数量的 double 数组参数
   *
   * 该函数用于读取关节限位、MIT 增益、目标速度和前馈力矩等按关节排列的参数。
   *
   * @param name 参数名
   *
   * @return std::array<double, kJointCount> 解析后的定长数组
   *
   * @throws std::runtime_error 当参数长度不是 kJointCount 时抛出异常
   */
  std::array<double, kJointCount> read_double_array(const std::string & name) const
  {
    const std::vector<double> values = get_parameter(name).as_double_array();
    if (values.size() != kJointCount) {
      throw std::runtime_error(name + " must contain 4 double values.");
    }

    std::array<double, kJointCount> array{};
    std::copy(values.begin(), values.end(), array.begin());
    return array;
  }

  /**
   * @brief 读取 double 参数并保证其不小于给定最小值
   *
   * 该函数用于频率、速度、阻尼、尺寸等必须为正或非负的配置项。若用户传入过小值，
   * 节点会使用 minimum，从而避免除零、负时长或无意义的物理尺寸。
   *
   * @param name 参数名
   * @param minimum 允许的最小值
   *
   * @return double max(parameter_value, minimum)
   */
  double positive_parameter(const std::string & name, const double minimum) const
  {
    return std::max(minimum, get_parameter(name).as_double());
  }

  /**
   * @brief 处理来自电机后端的关节状态反馈
   *
   * 该回调根据 joint_names_ 在 JointState 消息中查找目标关节，将对应位置读入 current_joints_。
   * 读取到的位置会先钳位到软件关节限位内，防止异常反馈污染后续 IK 初始种子。
   *
   * 当 4 个目标关节都至少收到过一次位置反馈后，have_joint_state_ 置为 true。之后新轨迹会以真实
   * 当前关节位置为起点，而不是以上一次命令目标为起点。
   *
   * @param msg ROS JointState 消息，通常来自 /dm_j4340/joint_states
   */
  void handle_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    for (std::size_t target_index = 0; target_index < kJointCount; ++target_index) {
      for (std::size_t source_index = 0; source_index < msg.name.size(); ++source_index) {
        if (msg.name[source_index] != joint_names_[target_index]) {
          continue;
        }
        if (source_index >= msg.position.size()) {
          continue;
        }

        current_joints_(static_cast<int>(target_index)) =
          std::clamp(msg.position[source_index], lower_limits_[target_index], upper_limits_[target_index]);
        joint_state_seen_[target_index] = true;
      }
    }

    have_joint_state_ = std::all_of(
      joint_state_seen_.begin(),
      joint_state_seen_.end(),
      [](const bool seen) {return seen;});
  }

  /**
   * @brief 处理外部发布的手动目标空间点
   *
   * 输入点被解释为 base_frame_ 坐标系下的末端执行器目标位置。当前节点只检查 frame_id 并给出警告，
   * 不执行 TF 坐标变换，因此上游发布者需要保证坐标已经转换到 base_link。
   *
   * 收到手动目标后，会中止正在执行的箱子任务，清空箱子关键点序列，然后调用普通位置 IK 并启动
   * 一段新的关节空间轨迹。
   *
   * @param msg 目标点消息，单位为米
   */
  void handle_target_point(const geometry_msgs::msg::PointStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Received target frame '%s', but this node expects '%s'. No TF transform is applied.",
        msg.header.frame_id.c_str(),
        base_frame_.c_str());
    }

    const Eigen::Vector3d target_position(msg.point.x, msg.point.y, msg.point.z);
    box_motion_active_ = false;
    box_motion_waiting_ = false;
    box_motion_targets_.clear();

    solve_and_start_target(
      target_position,
      false,
      "manual target",
      position_tolerance_m_,
      nullptr);
  }

  /**
   * @brief 处理箱子搬运动作启动信号
   *
   * 收到 std_msgs/msg/Empty 后，节点会根据箱子尺寸、平台高度、吸盘参数等生成一组任务关键点，
   * 并从第一个关键点开始逐段执行。每一段仍然通过 IK 求解为关节目标，再由定时器平滑插值执行。
   *
   * @param msg 空消息，仅作为触发事件使用
   */
  void handle_box_motion_start(const std_msgs::msg::Empty &)
  {
    box_motion_targets_ = make_box_motion_targets();
    box_motion_target_index_ = 0;
    box_motion_active_ = true;
    box_motion_waiting_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Suction box motion started (level-constrained): arm_to_box=%.3f m, platform_height=%.3f m, center_distance=%.3f m, size=%.3f x %.3f x %.3f m, mass=%.3f kg, base_yaw_limit=%.3f rad.",
      arm_to_box_distance_m_,
      platform_height_m_,
      box_center_distance_m_,
      box_width_m_,
      box_depth_m_,
      box_height_m_,
      box_mass_kg_,
      box_motion_base_yaw_limit_rad_);

    start_next_box_motion_target();
  }

  /**
   * @brief 处理“停止吸附并回原位”的动作启动信号
   *
   * 收到 std_msgs/msg/Empty 后，节点会立即发布 suction=false，确保吸盘状态先关闭；
   * 随后取消正在执行的箱子任务或段间等待，并启动一段关节空间轨迹回到 home_joint_positions。
   * home 位置不走 IK，直接使用配置的 4 个关节角，因此适合作为安全回零/复位动作。
   *
   * @param msg 空消息，仅作为触发事件使用
   */
  void handle_home_motion_start(const std_msgs::msg::Empty &)
  {
    publish_suction_command(false);

    box_motion_active_ = false;
    box_motion_waiting_ = false;
    box_motion_targets_.clear();
    box_motion_target_index_ = 0;

    start_trajectory(home_joints_);
    have_target_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Home motion started after suction release: q=[%.3f, %.3f, %.3f, %.3f].",
      home_joints_(0),
      home_joints_(1),
      home_joints_(2),
      home_joints_(3));
  }

  /**
   * @brief 对单个目标位置执行普通 IK，并在成功后启动轨迹
   *
   * 该函数是手动点位控制的核心入口，也可被其他只需要位置约束的任务复用。它负责：
   * 1. 根据重复目标死区过滤过近的重复命令。
   * 2. 选择 IK 初始种子，优先使用当前关节反馈，其次使用上一条目标命令。
   * 3. 按需使用传入的临时关节限位覆盖默认限位。
   * 4. 调用 solve_inverse_kinematics 求解目标关节角。
   * 5. 求解成功后调用 start_trajectory 进入平滑插值执行。
   *
   * @param target_position 末端目标位置，单位为米，坐标系为 base_frame_
   * @param ignore_repeat_deadband 是否忽略重复目标死区
   * @param label 日志中使用的目标标签
   * @param position_tolerance 允许的末端位置误差，单位为米
   * @param seed_override 可选 IK 初始种子，nullptr 表示自动选择
   * @param lower_limits_override 可选关节下限覆盖，nullptr 表示使用默认限位
   * @param upper_limits_override 可选关节上限覆盖，nullptr 表示使用默认限位
   *
   * @return bool true 表示 IK 成功且轨迹已启动，false 表示目标被忽略或 IK 失败
   */
  bool solve_and_start_target(
    const Eigen::Vector3d & target_position,
    const bool ignore_repeat_deadband,
    const std::string & label,
    const double position_tolerance,
    const JointVector * seed_override,
    const std::array<double, kJointCount> * lower_limits_override = nullptr,
    const std::array<double, kJointCount> * upper_limits_override = nullptr)
  {
    if (have_target_ && have_command_ &&
      !ignore_repeat_deadband &&
      (target_position - last_target_position_).norm() <= target_repeat_deadband_m_)
    {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignored repeated target inside %.4f m deadband.",
        target_repeat_deadband_m_);
      return false;
    }

    const JointVector seed =
      seed_override == nullptr ?
      (have_joint_state_ ? current_joints_ : active_goal_joints_) :
      *seed_override;
    const auto & solve_lower_limits =
      lower_limits_override == nullptr ? lower_limits_ : *lower_limits_override;
    const auto & solve_upper_limits =
      upper_limits_override == nullptr ? upper_limits_ : *upper_limits_override;

    JointVector solution;
    double final_error = 0.0;
    int iterations = 0;
    const bool solved = solve_inverse_kinematics(
      joint_origins_,
      joint_axes_,
      solve_lower_limits,
      solve_upper_limits,
      tool_offset_,
      target_position,
      seed,
      position_tolerance,
      max_iterations_,
      damping_,
      max_solver_step_rad_,
      solution,
      final_error,
      iterations);

    if (!solved) {
      RCLCPP_WARN(
        get_logger(),
        "IK failed for target [%.3f, %.3f, %.3f], best error %.4f m after %d iterations.",
        target_position.x(),
        target_position.y(),
        target_position.z(),
        final_error,
        iterations);
      box_motion_active_ = false;
      box_motion_waiting_ = false;
      return false;
    }

    start_trajectory(solution);
    last_target_position_ = target_position;
    have_target_ = true;

    RCLCPP_INFO(
      get_logger(),
      "IK %s accepted: q=[%.3f, %.3f, %.3f, %.3f], error=%.4f m, iterations=%d",
      label.c_str(),
      solution(0),
      solution(1),
      solution(2),
      solution(3),
      final_error,
      iterations);
    return true;
  }

  /**
   * @brief 生成箱子搬运动作的轨迹目标点序列
   * 
   * 该函数定义了箱子搬运操作的完整轨迹，包含5个关键位置点：
   * 1. 接近点 (approach)：安全距离外的起始位置
   * 2. 预接触点 (pre-contact)：接近箱子表面的位置
   * 3. 接触点 (contact)：与箱子表面接触的位置（含轻微推动）
   * 4. 提升点 (lift)：垂直提升后的位置
   * 5. 搬运点 (carry)：水平回撤后的搬运位置
   * 
   * 所有点都基于箱子参数和操作参数动态计算，确保安全性和有效性。
   * 
   * @return std::vector<Eigen::Vector3d> 箱子搬运轨迹的5个目标位置点
   */
  std::vector<Eigen::Vector3d> make_box_motion_targets() const
  {
    const double box_face_center_z_in_base =
      box_ground_z_m_ + box_height_m_ * box_touch_height_ratio_ - platform_height_m_;
    const double contact_z =
      std::max(box_min_touch_z_m_, box_face_center_z_in_base);
    const double lift_z = contact_z + box_lift_height_m_;
    const double face_x = box_suction_target_x_m_;
    const double approach_x = std::max(box_min_approach_x_m_, face_x - box_approach_offset_m_);
    const double pre_contact_x =
      std::max(box_min_approach_x_m_, face_x - box_suction_contact_offset_m_);
    const double contact_x = face_x + box_touch_push_m_;
    const double carry_x = std::max(box_min_approach_x_m_, face_x - box_lift_retract_m_);

    return {
      Eigen::Vector3d(approach_x, box_motion_y_m_, contact_z),
      Eigen::Vector3d(pre_contact_x, box_motion_y_m_, contact_z),
      Eigen::Vector3d(contact_x, box_motion_y_m_, contact_z),
      Eigen::Vector3d(contact_x, box_motion_y_m_, lift_z),
      Eigen::Vector3d(carry_x, box_motion_y_m_, lift_z),
    };
  }

  /**
   * @brief 为箱子任务构造带偏置的 IK 初始种子
   *
   * 为实现水平约束（wrist = elbow - shoulder），种子关节角需要满足此关系。
   * base_yaw 设为指向目标点的水平角度，shoulder/elbow 使用配置的经验偏置，
   * wrist 由 elbow - shoulder 直接算出并钳位到关节限位内。
   *
   * @param target_position 当前箱子关键点目标
   *
   * @return JointVector 用于水平约束 IK 的初始关节角
   */
  JointVector make_box_motion_seed(const Eigen::Vector3d & target_position) const
  {
    JointVector seed = have_joint_state_ ? current_joints_ : active_goal_joints_;
    seed(0) = std::atan2(target_position.y(), std::max(0.001, target_position.x()));
    seed(1) = box_motion_shoulder_bias_rad_;
    seed(2) = box_motion_elbow_bias_rad_;
    // Level constraint: wrist = elbow - shoulder
    seed(3) = std::clamp(seed(2) - seed(1), lower_limits_[3], upper_limits_[3]);
    return clamp_to_limits(seed, lower_limits_, upper_limits_);
  }

  /**
   * @brief 为箱子任务生成临时关节限位
   *
   * 当前实现只额外收紧 base_yaw 的搜索范围，使底座只能在箱子方向附近小幅调整。
   * 这样可以避免 IK 选择绕到侧面或背后的可行解，让吸盘动作保持“正面接近箱子”的任务语义。
   *
   * @param target_position 当前箱子关键点目标
   * @param lower_limits 输出的临时关节下限
   * @param upper_limits 输出的临时关节上限
   */
  void make_box_motion_joint_limits(
    const Eigen::Vector3d & target_position,
    std::array<double, kJointCount> & lower_limits,
    std::array<double, kJointCount> & upper_limits) const
  {
    lower_limits = lower_limits_;
    upper_limits = upper_limits_;

    const double yaw_center = std::atan2(target_position.y(), std::max(0.001, target_position.x()));
    lower_limits[0] = std::max(lower_limits[0], yaw_center - box_motion_base_yaw_limit_rad_);
    upper_limits[0] = std::min(upper_limits[0], yaw_center + box_motion_base_yaw_limit_rad_);
  }

  /**
   * @brief 计算箱子任务期望的工具 x 轴朝向
   *
   * 对吸盘侧面接触而言，希望工具坐标系局部 x 轴大致朝向目标点的水平径向方向。
   * 因此该函数取目标点在 XY 平面上的方向并归一化，忽略 z 分量，避免提升阶段因为高度变化改变
   * 吸盘朝向约束。
   *
   * @param target_position 当前箱子关键点目标
   *
   * @return Eigen::Vector3d 归一化后的期望工具 x 轴方向
   */
  Eigen::Vector3d box_motion_tool_direction(const Eigen::Vector3d & target_position) const
  {
    Eigen::Vector3d direction(target_position.x(), target_position.y(), 0.0);
    if (direction.squaredNorm() < 1.0e-12) {
      return Eigen::Vector3d::UnitX();
    }
    return direction.normalized();
  }

  /**
   * @brief 对箱子任务的单个关键点执行水平约束 IK
   *
   * 箱子任务要求末端执行器始终保持水平（wrist = elbow - shoulder），
   * 这样才能让吸盘在接近、接触、提升、搬运全过程中不倾斜。
   * 该函数使用 solve_inverse_kinematics_level 在 3 自由度约化空间求解，
   * 自动保证末端水平。成功后启动关节空间轨迹，失败后停止整个箱子任务。
   *
   * @param target_position 当前箱子关键点目标，单位为米
   * @param label 日志中使用的阶段标签
   * @param seed IK 初始种子
   * @param lower_limits 当前阶段使用的关节下限
   * @param upper_limits 当前阶段使用的关节上限
   *
   * @return bool true 表示 IK 成功且轨迹已启动，false 表示 IK 失败并已停止任务
   */
  bool solve_and_start_box_target(
    const Eigen::Vector3d & target_position,
    const std::string & label,
    const JointVector & seed,
    const std::array<double, kJointCount> & lower_limits,
    const std::array<double, kJointCount> & upper_limits)
  {
    JointVector solution;
    double final_error = 0.0;
    int iterations = 0;
    const bool solved = solve_inverse_kinematics_level(
      joint_origins_,
      joint_axes_,
      lower_limits,
      upper_limits,
      tool_offset_,
      target_position,
      seed,
      box_motion_position_tolerance_m_,
      max_iterations_,
      damping_,
      max_solver_step_rad_,
      solution,
      final_error,
      iterations);

    if (!solved) {
      RCLCPP_WARN(
        get_logger(),
        "IK %s failed for target [%.3f, %.3f, %.3f], best error %.4f m after %d iterations.",
        label.c_str(),
        target_position.x(),
        target_position.y(),
        target_position.z(),
        final_error,
        iterations);
      box_motion_active_ = false;
      box_motion_waiting_ = false;
      return false;
    }

    start_trajectory(solution);
    last_target_position_ = target_position;
    have_target_ = true;

    RCLCPP_INFO(
      get_logger(),
      "IK %s accepted: q=[%.3f, %.3f, %.3f, %.3f], error=%.4f m, iterations=%d (level-constrained: wrist=elbow-shoulder=%.3f)",
      label.c_str(),
      solution(0),
      solution(1),
      solution(2),
      solution(3),
      final_error,
      iterations,
      solution(2) - solution(1));
    return true;
  }

  /**
   * @brief 启动箱子任务中的下一个关键点
   *
   * 该函数由任务启动回调和定时器在每段轨迹完成后的等待结束时调用。它负责检查任务是否结束，
   * 取出当前关键点，构造专用 IK 种子和临时关节限位，然后调用 solve_and_start_box_target。
   */
  void start_next_box_motion_target()
  {
    if (!box_motion_active_) {
      return;
    }

    if (box_motion_target_index_ >= box_motion_targets_.size()) {
      box_motion_active_ = false;
      box_motion_waiting_ = false;
      RCLCPP_INFO(get_logger(), "Box motion finished.");
      return;
    }

    const Eigen::Vector3d target_position = box_motion_targets_[box_motion_target_index_];
    const JointVector seed = make_box_motion_seed(target_position);
    std::array<double, kJointCount> box_lower_limits{};
    std::array<double, kJointCount> box_upper_limits{};
    make_box_motion_joint_limits(target_position, box_lower_limits, box_upper_limits);
    const std::string label =
      "box step " + std::to_string(box_motion_target_index_ + 1) + "/" +
      std::to_string(box_motion_targets_.size());

    if (!solve_and_start_box_target(
        target_position,
        label,
        seed,
        box_lower_limits,
        box_upper_limits))
    {
      RCLCPP_WARN(
        get_logger(),
        "Box motion stopped at step %zu.",
        box_motion_target_index_ + 1);
    }
  }

  /**
   * @brief 根据目标关节角启动一段关节空间轨迹
   *
   * 轨迹起点优先取真实关节反馈 current_joints_；如果还没有收到完整反馈，则使用上一段目标
   * active_goal_joints_ 作为连续起点。轨迹终点会先被关节限位钳位。
   *
   * 轨迹时长由最大单关节角度变化量和 max_joint_speed_rad_s_ 决定，并受
   * min_trajectory_duration_s_ 下限保护，避免小距离目标形成过快阶跃。
   *
   * @param goal IK 求得的目标关节角，单位为弧度
   */
  void start_trajectory(const JointVector & goal)
  {
    active_start_joints_ = have_joint_state_ ? current_joints_ : active_goal_joints_;
    active_goal_joints_ = clamp_to_limits(goal, lower_limits_, upper_limits_);
    command_joints_ = active_start_joints_;

    double max_delta = 0.0;
    for (int index = 0; index < command_joints_.size(); ++index) {
      max_delta = std::max(max_delta, std::abs(active_goal_joints_(index) - active_start_joints_(index)));
    }

    trajectory_duration_s_ = std::max(
      min_trajectory_duration_s_,
      max_delta / max_joint_speed_rad_s_);
    trajectory_start_time_ = now();
    active_trajectory_ = true;
    have_command_ = true;
  }

  /**
   * @brief 控制定时器回调，负责轨迹插值、命令发布和箱子任务推进
   *
   * 该回调以 command_rate_hz_ 频率执行。运行逻辑如下：
   * 1. 如果箱子任务正在段间等待，并且等待时间已到，则启动下一个关键点。
   * 2. 如果当前有活动轨迹，根据 elapsed / duration 计算 smoothstep 插值比例。
   * 3. 将插值得到的 command_joints_ 发布为 MIT 命令。
   * 4. 轨迹结束后根据 hold_last_command_ 决定是否继续保持目标。
   * 5. 若箱子任务刚完成接触点，则开启吸盘，并等待吸附稳定时间后继续下一段。
   */
  void publish_command_timer()
  {
    if (box_motion_active_ && box_motion_waiting_ && !active_trajectory_) {
      if ((now() - box_motion_wait_until_).seconds() >= 0.0) {
        box_motion_waiting_ = false;
        start_next_box_motion_target();
      }
    }

    if (!have_command_) {
      return;
    }

    bool trajectory_finished_now = false;
    if (active_trajectory_) {
      const double elapsed_s = (now() - trajectory_start_time_).seconds();
      const double ratio =
        trajectory_duration_s_ <= 0.0 ? 1.0 : elapsed_s / trajectory_duration_s_;
      const double blend = smoothstep(ratio);
      command_joints_ = active_start_joints_ + blend * (active_goal_joints_ - active_start_joints_);

      if (ratio >= 1.0) {
        active_trajectory_ = false;
        command_joints_ = active_goal_joints_;
        trajectory_finished_now = true;
      }
    } else if (!hold_last_command_) {
      return;
    }

    publish_mit_command(command_joints_, !active_trajectory_);

    if (trajectory_finished_now && box_motion_active_) {
      const std::size_t completed_step = box_motion_target_index_;
      ++box_motion_target_index_;
      if (completed_step == box_suction_contact_step_index_) {
        publish_suction_command(true);
      }
      box_motion_waiting_ = true;
      const double wait_s =
        completed_step == box_suction_contact_step_index_ ?
        box_suction_settle_s_ :
        box_motion_step_wait_s_;
      box_motion_wait_until_ = now() + rclcpp::Duration::from_seconds(wait_s);
    }
  }

  /**
   * @brief 发布吸盘开关命令
   *
   * 当前箱子任务在接触点轨迹完成后会调用该函数打开吸盘。发布的是 Bool 消息，
   * true 表示开启吸盘，false 表示关闭吸盘。
   *
   * @param enabled 期望吸盘状态
   */
  void publish_suction_command(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    suction_command_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Suction command: %s.", enabled ? "on" : "off");
  }

  /**
   * @brief 发布 4 个关节的 MIT 控制命令
   *
   * 消息格式固定为 [p, v, kp, kd, t_ff] * 4：
   * - p：目标关节位置，单位为弧度。
   * - v：目标关节速度，当前默认配置为 0。
   * - kp：位置刚度，运动阶段使用 kp_，保持阶段使用 hold_kp_。
   * - kd：速度阻尼，运动阶段使用 kd_，保持阶段使用 hold_kd_。
   * - t_ff：力矩前馈，当前默认配置为 0，后续可用于重力或负载补偿。
   *
   * @param joints 当前要下发的目标关节角
   * @param use_hold_gains true 表示使用保持阶段增益，false 表示使用运动阶段增益
   */
  void publish_mit_command(const JointVector & joints, const bool use_hold_gains)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(kJointCount * 5);

    const auto & active_kp = use_hold_gains ? hold_kp_ : kp_;
    const auto & active_kd = use_hold_gains ? hold_kd_ : kd_;

    for (std::size_t index = 0; index < kJointCount; ++index) {
      msg.data.push_back(static_cast<float>(joints(static_cast<int>(index))));
      msg.data.push_back(static_cast<float>(command_velocity_[index]));
      msg.data.push_back(static_cast<float>(active_kp[index]));
      msg.data.push_back(static_cast<float>(active_kd[index]));
      msg.data.push_back(static_cast<float>(torque_ff_[index]));
    }

    mit_command_publisher_->publish(msg);
  }

  // -------- 参数缓存：关节映射、话题名称和坐标系 --------
  //
  // 这些字段由 declare_parameters()/load_parameters() 初始化，用于把外部 ROS 接口和内部
  // 4 自由度关节顺序绑定起来。joint_names_ 的顺序同时决定 JointState 读取顺序和 MIT 命令发布顺序。
  std::array<std::string, kJointCount> joint_names_{};
  std::string target_point_topic_;
  std::string box_motion_start_topic_;
  std::string home_motion_start_topic_;
  std::string suction_command_topic_;
  std::string joint_states_topic_;
  std::string mit_command_topic_;
  std::string base_frame_;

  // -------- 参数缓存：机械臂几何和底层 MIT 控制量 --------
  //
  // joint_origins_、joint_axes_ 和 tool_offset_ 共同定义运动学模型。lower_limits_/upper_limits_
  // 既用于 IK 搜索约束，也用于最终目标关节角钳位。Kp/Kd 分为运动阶段和保持阶段两套，
  // 便于到位后降低刚度、提高阻尼来减少静止抖动。
  std::array<Eigen::Vector3d, kJointCount> joint_origins_{};
  std::array<Eigen::Vector3d, kJointCount> joint_axes_{};
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  Eigen::Vector3d tool_offset_{Eigen::Vector3d::Zero()};
  std::array<double, kJointCount> kp_{};
  std::array<double, kJointCount> kd_{};
  std::array<double, kJointCount> hold_kp_{};
  std::array<double, kJointCount> hold_kd_{};
  std::array<double, kJointCount> command_velocity_{};
  std::array<double, kJointCount> torque_ff_{};
  JointVector home_joints_{JointVector::Zero()};

  // -------- 参数缓存：IK 求解与关节空间轨迹控制 --------
  //
  // 这些参数控制目标点求解精度、数值 IK 的稳定性、轨迹插值速度和命令发布策略。
  // position_tolerance_m_ 越小越精确但越难收敛；damping_ 越大越抗奇异但响应更保守；
  // max_joint_speed_rad_s_ 和 min_trajectory_duration_s_ 共同决定平滑运动时间。
  double position_tolerance_m_{0.005};
  int max_iterations_{100};
  double damping_{0.04};
  double max_solver_step_rad_{0.12};
  double target_repeat_deadband_m_{0.002};
  double command_rate_hz_{50.0};
  double max_joint_speed_rad_s_{0.18};
  double min_trajectory_duration_s_{1.2};
  bool hold_last_command_{true};

  // -------- 参数缓存：箱子吸取/搬运动作模型 --------
  //
  // 这组参数描述箱子在 base_link 坐标系下的任务几何关系，以及吸盘接近、接触、提升和回收动作。
  // 当前任务规划是规则式关键点序列，不包含全局避障或碰撞检测，因此这些数值需要和实际平台、
  // 箱子尺寸、机械臂可达空间匹配。
  double box_center_distance_m_{0.75};
  double arm_to_box_distance_m_{0.75};
  double platform_height_m_{0.16};
  double box_ground_z_m_{0.0};
  double box_width_m_{0.25};
  double box_depth_m_{0.25};
  double box_height_m_{0.25};
  double box_mass_kg_{0.5};
  double box_motion_y_m_{0.0};
  double box_motion_base_yaw_limit_rad_{0.50};
  double box_motion_tool_direction_tolerance_{0.35};
  double box_motion_tool_direction_weight_{0.12};
  double box_approach_offset_m_{0.06};
  double box_min_approach_x_m_{0.14};
  double box_touch_push_m_{0.005};
  double box_touch_height_ratio_{0.5};
  double box_min_touch_z_m_{-0.06};
  double box_suction_target_x_m_{0.625};
  double box_suction_approach_clearance_m_{0.06};
  double box_suction_contact_offset_m_{0.01};
  double box_suction_settle_s_{0.8};
  double box_lift_height_m_{0.08};
  double box_lift_retract_m_{0.04};
  double box_motion_step_wait_s_{0.5};
  double box_motion_position_tolerance_m_{0.02};
  double box_motion_shoulder_bias_rad_{0.85};
  double box_motion_elbow_bias_rad_{0.4};
  double box_motion_wrist_bias_rad_{-0.6};

  // -------- 运行时状态：关节反馈、轨迹插值和任务进度 --------
  //
  // current_joints_ 来自真实或仿真的 JointState 反馈；active_start_joints_ 和 active_goal_joints_
  // 定义当前轨迹段端点；command_joints_ 是定时器当前周期插值得到的命令位置。
  // box_motion_* 字段记录箱子任务是否正在执行、是否处于段间等待，以及当前关键点索引。
  JointVector current_joints_{JointVector::Zero()};
  JointVector active_start_joints_{JointVector::Zero()};
  JointVector active_goal_joints_{JointVector::Zero()};
  JointVector command_joints_{JointVector::Zero()};
  std::array<bool, kJointCount> joint_state_seen_{};
  bool have_joint_state_{false};
  bool have_command_{false};
  bool active_trajectory_{false};
  bool have_target_{false};
  bool box_motion_active_{false};
  bool box_motion_waiting_{false};
  std::size_t box_motion_target_index_{0};
  std::size_t box_suction_contact_step_index_{2};
  double trajectory_duration_s_{0.0};
  Eigen::Vector3d last_target_position_{Eigen::Vector3d::Zero()};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time box_motion_wait_until_{0, 0, RCL_ROS_TIME};
  std::vector<Eigen::Vector3d> box_motion_targets_;

  // -------- ROS 通信对象 --------
  //
  // 订阅器负责接收上层目标和底层关节反馈；发布器负责向底层后端输出 MIT 命令和吸盘开关；
  // command_timer_ 是本节点唯一的周期控制循环。
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_motion_start_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr home_motion_start_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mit_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_command_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace arm_controller

/**
 * @brief arm_controller_node 可执行程序入口
 *
 * 初始化 ROS 2，创建 ArmControllerNode 并进入 rclcpp::spin() 事件循环。节点退出时调用
 * rclcpp::shutdown() 释放 ROS 资源。
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 *
 * @return int 进程退出码，0 表示正常退出
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_controller::ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
