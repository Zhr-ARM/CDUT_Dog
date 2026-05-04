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

namespace arm_controller
{

using JointVector = Eigen::Matrix<double, 4, 1>;

Eigen::Vector3d compute_end_effector_position(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const JointVector & joint_positions,
  const Eigen::Vector3d & tool_offset);

bool solve_inverse_kinematics(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
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

bool solve_inverse_kinematics_with_tool_direction(
  const std::array<Eigen::Vector3d, 4> & joint_origins,
  const std::array<Eigen::Vector3d, 4> & joint_axes,
  const std::array<double, 4> & lower_limits,
  const std::array<double, 4> & upper_limits,
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

namespace
{

constexpr std::size_t kJointCount = 4;

template<typename T>
T clamp_value(const T value, const T lower, const T upper)
{
  return std::max(lower, std::min(value, upper));
}

double smoothstep(const double t)
{
  const double clamped = clamp_value(t, 0.0, 1.0);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

}  // namespace

class ArmControllerNode : public rclcpp::Node
{
public:
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
      "arm_controller is ready: target=%s, box_start=%s, joint_states=%s, mit_commands=%s",
      target_point_topic_.c_str(),
      box_motion_start_topic_.c_str(),
      joint_states_topic_.c_str(),
      mit_command_topic_.c_str());
  }

private:
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

  void load_parameters()
  {
    target_point_topic_ = get_parameter("target_point_topic").as_string();
    box_motion_start_topic_ = get_parameter("box_motion_start_topic").as_string();
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
    box_motion_base_yaw_limit_rad_ = clamp_value(
      positive_parameter("box_motion_base_yaw_limit_rad", 0.01),
      0.01,
      3.14);
    box_motion_tool_direction_tolerance_ = clamp_value(
      positive_parameter("box_motion_tool_direction_tolerance", 0.01),
      0.01,
      2.0);
    box_motion_tool_direction_weight_ =
      positive_parameter("box_motion_tool_direction_weight", 0.001);
    box_approach_offset_m_ = std::max(0.0, get_parameter("box_approach_offset_m").as_double());
    box_min_approach_x_m_ = positive_parameter("box_min_approach_x_m", 0.01);
    box_touch_push_m_ = std::max(0.0, get_parameter("box_touch_push_m").as_double());
    box_touch_height_ratio_ =
      clamp_value(get_parameter("box_touch_height_ratio").as_double(), 0.05, 0.95);
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
      clamp_value(
      get_parameter("box_motion_shoulder_bias_rad").as_double(),
      lower_limits_[1],
      upper_limits_[1]);
    box_motion_elbow_bias_rad_ =
      clamp_value(
      get_parameter("box_motion_elbow_bias_rad").as_double(),
      lower_limits_[2],
      upper_limits_[2]);
    box_motion_wrist_bias_rad_ =
      clamp_value(
      get_parameter("box_motion_wrist_bias_rad").as_double(),
      lower_limits_[3],
      upper_limits_[3]);

    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (lower_limits_[index] > upper_limits_[index]) {
        throw std::runtime_error("joint lower limit must not be greater than upper limit.");
      }
      current_joints_(static_cast<int>(index)) =
        0.5 * (lower_limits_[index] + upper_limits_[index]);
    }
  }

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

  Eigen::Vector3d read_vector3(const std::string & name) const
  {
    const std::vector<double> values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain 3 double values.");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

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

  double positive_parameter(const std::string & name, const double minimum) const
  {
    return std::max(minimum, get_parameter(name).as_double());
  }

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
          clamp_value(msg.position[source_index], lower_limits_[target_index], upper_limits_[target_index]);
        joint_state_seen_[target_index] = true;
      }
    }

    have_joint_state_ = std::all_of(
      joint_state_seen_.begin(),
      joint_state_seen_.end(),
      [](const bool seen) {return seen;});
  }

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

  void handle_box_motion_start(const std_msgs::msg::Empty &)
  {
    box_motion_targets_ = make_box_motion_targets();
    box_motion_target_index_ = 0;
    box_motion_active_ = true;
    box_motion_waiting_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Suction box motion started: arm_to_box=%.3f m, platform_height=%.3f m, center_distance=%.3f m, size=%.3f x %.3f x %.3f m, mass=%.3f kg, base_yaw_limit=%.3f rad, tool_direction_tolerance=%.3f.",
      arm_to_box_distance_m_,
      platform_height_m_,
      box_center_distance_m_,
      box_width_m_,
      box_depth_m_,
      box_height_m_,
      box_mass_kg_,
      box_motion_base_yaw_limit_rad_,
      box_motion_tool_direction_tolerance_);

    start_next_box_motion_target();
  }

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

  JointVector make_box_motion_seed(const Eigen::Vector3d & target_position) const
  {
    JointVector seed = have_joint_state_ ? current_joints_ : active_goal_joints_;
    seed(0) = std::atan2(target_position.y(), std::max(0.001, target_position.x()));
    seed(1) = box_motion_shoulder_bias_rad_;
    seed(2) = box_motion_elbow_bias_rad_;
    seed(3) = box_motion_wrist_bias_rad_;
    return clamp_joint_vector(seed);
  }

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

  Eigen::Vector3d box_motion_tool_direction(const Eigen::Vector3d & target_position) const
  {
    Eigen::Vector3d direction(target_position.x(), target_position.y(), 0.0);
    if (direction.squaredNorm() < 1.0e-12) {
      return Eigen::Vector3d::UnitX();
    }
    return direction.normalized();
  }

  bool solve_and_start_box_target(
    const Eigen::Vector3d & target_position,
    const std::string & label,
    const JointVector & seed,
    const std::array<double, kJointCount> & lower_limits,
    const std::array<double, kJointCount> & upper_limits)
  {
    JointVector solution;
    double final_position_error = 0.0;
    double final_direction_error = 0.0;
    int iterations = 0;
    const Eigen::Vector3d target_direction = box_motion_tool_direction(target_position);
    const bool solved = solve_inverse_kinematics_with_tool_direction(
      joint_origins_,
      joint_axes_,
      lower_limits,
      upper_limits,
      tool_offset_,
      target_position,
      target_direction,
      seed,
      box_motion_position_tolerance_m_,
      box_motion_tool_direction_tolerance_,
      box_motion_tool_direction_weight_,
      max_iterations_,
      damping_,
      max_solver_step_rad_,
      solution,
      final_position_error,
      final_direction_error,
      iterations);

    if (!solved) {
      RCLCPP_WARN(
        get_logger(),
        "IK %s failed for target [%.3f, %.3f, %.3f], best position error %.4f m, direction error %.4f after %d iterations.",
        label.c_str(),
        target_position.x(),
        target_position.y(),
        target_position.z(),
        final_position_error,
        final_direction_error,
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
      "IK %s accepted: q=[%.3f, %.3f, %.3f, %.3f], position_error=%.4f m, direction_error=%.4f, iterations=%d",
      label.c_str(),
      solution(0),
      solution(1),
      solution(2),
      solution(3),
      final_position_error,
      final_direction_error,
      iterations);
    return true;
  }

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

  void start_trajectory(const JointVector & goal)
  {
    active_start_joints_ = have_joint_state_ ? current_joints_ : active_goal_joints_;
    active_goal_joints_ = clamp_joint_vector(goal);
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

  JointVector clamp_joint_vector(const JointVector & joints) const
  {
    JointVector clamped = joints;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      clamped(static_cast<int>(index)) =
        clamp_value(clamped(static_cast<int>(index)), lower_limits_[index], upper_limits_[index]);
    }
    return clamped;
  }

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

  void publish_suction_command(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    suction_command_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Suction command: %s.", enabled ? "on" : "off");
  }

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

  std::array<std::string, kJointCount> joint_names_{};
  std::string target_point_topic_;
  std::string box_motion_start_topic_;
  std::string suction_command_topic_;
  std::string joint_states_topic_;
  std::string mit_command_topic_;
  std::string base_frame_;

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

  double position_tolerance_m_{0.005};
  int max_iterations_{100};
  double damping_{0.04};
  double max_solver_step_rad_{0.12};
  double target_repeat_deadband_m_{0.002};
  double command_rate_hz_{50.0};
  double max_joint_speed_rad_s_{0.18};
  double min_trajectory_duration_s_{1.2};
  bool hold_last_command_{true};
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

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr box_motion_start_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mit_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_command_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace arm_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_controller::ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
