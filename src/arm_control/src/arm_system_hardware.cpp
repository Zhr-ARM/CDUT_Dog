#include "arm_control/arm_system_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_control {
namespace {

using dm_j4340_motor_driver::ControlMode;
using dm_j4340_motor_driver::Register;

const rclcpp::Logger kLogger = rclcpp::get_logger("ArmSystemHardware");

template <typename ParameterMap>
std::string get_string(const ParameterMap &parameters, const std::string &name,
                       const std::string &default_value) {
  const auto it = parameters.find(name);
  if (it == parameters.end() || it->second.empty()) {
    return default_value;
  }
  return it->second;
}

template <typename ParameterMap>
int get_int(const ParameterMap &parameters, const std::string &name,
            int default_value) {
  const auto text = get_string(parameters, name, "");
  if (text.empty()) {
    return default_value;
  }
  return std::stoi(text);
}

template <typename ParameterMap>
double get_double(const ParameterMap &parameters, const std::string &name,
                  double default_value) {
  const auto text = get_string(parameters, name, "");
  if (text.empty()) {
    return default_value;
  }
  return std::stod(text);
}

template <typename ParameterMap>
bool get_bool(const ParameterMap &parameters, const std::string &name,
              bool default_value) {
  std::string text = get_string(parameters, name, "");
  if (text.empty()) {
    return default_value;
  }
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text == "true" || text == "1" || text == "yes" || text == "on";
}

bool parse_optional_double(const std::string &text, double &value) {
  if (text.empty()) {
    return false;
  }
  value = std::stod(text);
  return true;
}

ControlMode control_mode_from_string(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "mit") {
    return ControlMode::MIT;
  }
  if (mode == "position_velocity" || mode == "pos_vel" || mode == "posvel") {
    return ControlMode::PositionVelocity;
  }
  if (mode == "velocity" || mode == "vel") {
    return ControlMode::Velocity;
  }
  if (mode == "force_position" || mode == "pos_force" || mode == "posi") {
    return ControlMode::ForcePosition;
  }
  throw std::invalid_argument("Unsupported DM-J4340 control mode: " + mode);
}

} // namespace

ArmSystemHardware::~ArmSystemHardware() {
  if (bus_.is_open()) {
    run_stop_sequence(false, enable_on_activate_ || disable_on_shutdown_ ||
                                 disable_on_deactivate_);
    bus_.close();
  }
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(kLogger, "ArmSystemHardware 初始化失败。");
    return CallbackReturn::ERROR;
  }

  const auto &hardware_params = info_.hardware_parameters;
  serial_port_ = get_string(hardware_params, "serial_port", "/dev/arm_motor");
  serial_baudrate_ = get_int(hardware_params, "serial_baudrate", 921600);
  serial_open_retry_count_ =
      std::max(0, get_int(hardware_params, "serial_open_retry_count", 10));
  serial_open_retry_delay_ms_ =
      std::max(0, get_int(hardware_params, "serial_open_retry_delay_ms", 300));
  timeout_ms_ = get_int(hardware_params, "timeout_ms", 5);
  retry_count_ = get_int(hardware_params, "retry_count", 1);
  usbcan_command_ =
      static_cast<uint8_t>(get_int(hardware_params, "usbcan_command", 0x03));
  set_can_bitrate_on_open_ =
      get_bool(hardware_params, "set_can_bitrate_on_open", false);
  can_bitrate_index_ =
      static_cast<uint8_t>(get_int(hardware_params, "can_bitrate_index", 0));

  control_mode_ = control_mode_from_string(
      get_string(hardware_params, "control_mode", "position_velocity"));
  enable_on_activate_ = get_bool(hardware_params, "enable_on_activate", true);
  clear_error_on_activate_ =
      get_bool(hardware_params, "clear_error_on_activate", false);
  save_zero_on_activate_ =
      get_bool(hardware_params, "save_zero_on_activate", false);
  switch_mode_on_activate_ =
      get_bool(hardware_params, "switch_mode_on_activate", true);
  require_register_ack_ =
      get_bool(hardware_params, "require_register_ack", true);
  require_motors_on_configure_ =
      get_bool(hardware_params, "require_motors_on_configure", true);
  require_feedback_on_activate_ =
      get_bool(hardware_params, "require_feedback_on_activate", true);
  disable_on_deactivate_ =
      get_bool(hardware_params, "disable_on_deactivate", true);
  disable_on_shutdown_ =
      get_bool(hardware_params, "disable_on_shutdown", true);
  read_feedback_in_read_ =
      get_bool(hardware_params, "read_feedback_in_read", false);
  home_on_deactivate_ =
      get_bool(hardware_params, "home_on_deactivate", false);
  home_on_shutdown_ =
      get_bool(hardware_params, "home_on_shutdown", false);
  shutdown_home_duration_s_ =
      std::max(0.0, get_double(hardware_params, "shutdown_home_duration_s", 4.0));
  shutdown_home_command_rate_hz_ =
      std::max(1.0, get_double(hardware_params, "shutdown_home_command_rate_hz", 50.0));
  disable_command_repeat_count_ =
      std::max(1, get_int(hardware_params, "disable_command_repeat_count", 3));
  disable_command_repeat_delay_ms_ =
      std::max(0, get_int(hardware_params, "disable_command_repeat_delay_ms", 20));

  joint_names_.clear();
  joint_configs_.clear();

  for (size_t index = 0; index < info_.joints.size(); ++index) {
    const auto &joint = info_.joints[index];
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(kLogger, "关节 '%s' 必须且只能有一个 position 命令接口。",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(kLogger,
                   "关节 '%s' 必须有两个状态接口：position 和 velocity。",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    JointConfig joint_config;
    joint_config.motor_id = static_cast<uint16_t>(
        get_int(joint.parameters, "motor_id", static_cast<int>(index)));
    joint_config.direction = get_double(joint.parameters, "direction", 1.0);
    joint_config.zero_offset = get_double(joint.parameters, "zero_offset", 0.0);
    joint_config.gear_ratio = get_double(joint.parameters, "gear_ratio", 1.0);
    joint_config.velocity_limit =
        get_double(joint.parameters, "velocity_limit", 1.0);
    joint_config.force_position_velocity_limit = static_cast<uint16_t>(
        get_int(joint.parameters, "force_position_velocity_limit", 1000));
    joint_config.force_position_current_limit = static_cast<uint16_t>(
        get_int(joint.parameters, "force_position_current_limit", 1000));
    if (std::abs(joint_config.direction) < 1e-9 ||
        std::abs(joint_config.gear_ratio) < 1e-9) {
      RCLCPP_ERROR(kLogger, "关节 '%s' 的 direction 和 gear_ratio 不能为 0。",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    joint_config.has_lower_limit = parse_optional_double(
        joint.command_interfaces[0].min, joint_config.lower_limit);
    joint_config.has_upper_limit = parse_optional_double(
        joint.command_interfaces[0].max, joint_config.upper_limit);

    joint_names_.push_back(joint.name);
    joint_configs_.push_back(joint_config);
  }

  const size_t joint_count = joint_names_.size();
  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  previous_hw_positions_.assign(joint_count, 0.0);

  RCLCPP_INFO(kLogger, "DM-J4340 硬件接口初始化完成：%zu 个关节，串口 %s。",
              joint_count, serial_port_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 2);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
  bool serial_opened = false;
  std::string open_error;
  for (int attempt = 0; attempt <= serial_open_retry_count_; ++attempt) {
    try {
      bus_.open(serial_port_, serial_baudrate_);
      serial_opened = true;
      break;
    } catch (const std::exception &ex) {
      open_error = ex.what();
      RCLCPP_WARN(
          kLogger, "打开 DM-J4340 USB-CAN 串口失败，第 %d/%d 次：%s",
          attempt + 1, serial_open_retry_count_ + 1, ex.what());
      if (attempt < serial_open_retry_count_) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(serial_open_retry_delay_ms_));
      }
    }
  }

  if (!serial_opened) {
    RCLCPP_ERROR(kLogger, "打开 DM-J4340 USB-CAN 串口最终失败：%s",
                 open_error.c_str());
    return CallbackReturn::ERROR;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  bus_.discard_pending();

  if (set_can_bitrate_on_open_ && !bus_.set_can_bitrate(can_bitrate_index_)) {
    RCLCPP_ERROR(kLogger, "USB-CAN 适配器 CAN 波特率配置命令发送失败。");
    bus_.close();
    return CallbackReturn::ERROR;
  }

  bool all_motors_online = true;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto motor_id = joint_configs_[i].motor_id;
    const auto feedback = read_motor_feedback(motor_id);
    if (feedback.has_value()) {
      hw_positions_[i] = motor_to_joint_position(i, feedback->position);
      hw_velocities_[i] = motor_to_joint_velocity(i, feedback->velocity);
      hw_commands_[i] = hw_positions_[i];
      previous_hw_positions_[i] = hw_positions_[i];
      RCLCPP_INFO(kLogger, "电机 %u 在线，关节 %s 初始位置 %.4f rad。",
                  motor_id, joint_names_[i].c_str(), hw_positions_[i]);
      continue;
    }

    const auto esc_id = read_motor_register(motor_id, Register::ESC_ID);
    if (esc_id.has_value()) {
      RCLCPP_INFO(
          kLogger, "电机 %u 在线，读取 ESC_ID=%s。", motor_id,
          dm_j4340_motor_driver::format_register_value(*esc_id).c_str());
      continue;
    }

    all_motors_online = false;
    RCLCPP_ERROR(kLogger, "未检测到关节 %s 对应的电机 ID %u。",
                 joint_names_[i].c_str(), motor_id);
  }

  if (!all_motors_online && require_motors_on_configure_) {
    bus_.close();
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
  if (!bus_.is_open()) {
    RCLCPP_ERROR(kLogger, "DM-J4340 串口未打开，无法激活硬件接口。");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto motor_id = joint_configs_[i].motor_id;
    if (clear_error_on_activate_ &&
        !send_frame(motor_id,
                    dm_j4340_motor_driver::make_clear_error_command())) {
      RCLCPP_WARN(kLogger, "电机 %u 错误清除命令发送失败。", motor_id);
    }

    if (save_zero_on_activate_ && !zero_saved_this_session_) {
      if (!send_frame(motor_id,
                      dm_j4340_motor_driver::make_save_zero_command())) {
        RCLCPP_ERROR(kLogger, "电机 %u 保存当前位置为零点命令发送失败。",
                     motor_id);
        return CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(kLogger, "电机 %u 已保存当前位置为零点。", motor_id);
    }

    if (switch_mode_on_activate_ &&
        !write_motor_register_uint32(motor_id, Register::CTRL_MODE,
                                     static_cast<uint32_t>(control_mode_),
                                     require_register_ack_)) {
      RCLCPP_ERROR(kLogger, "电机 %u 控制模式切换失败。", motor_id);
      return CallbackReturn::ERROR;
    }

    if (enable_on_activate_ &&
        !send_frame(motor_id, dm_j4340_motor_driver::make_enable_command())) {
      RCLCPP_ERROR(kLogger, "电机 %u 使能命令发送失败。", motor_id);
      return CallbackReturn::ERROR;
    }

    const auto feedback = read_motor_feedback(motor_id);
    if (feedback.has_value()) {
      hw_positions_[i] = motor_to_joint_position(i, feedback->position);
      hw_velocities_[i] = motor_to_joint_velocity(i, feedback->velocity);
    } else if (require_feedback_on_activate_) {
      RCLCPP_ERROR(kLogger,
                   "激活时未读取到电机 %u 的位置反馈，拒绝发送控制命令。",
                   motor_id);
      return CallbackReturn::ERROR;
    }
    hw_commands_[i] = hw_positions_[i];
    previous_hw_positions_[i] = hw_positions_[i];
  }

  if (save_zero_on_activate_) {
    zero_saved_this_session_ = true;
  }

  active_ = true;
  stop_sequence_done_ = false;
  return write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0)) ==
                 hardware_interface::return_type::OK
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
  if (home_on_deactivate_ || disable_on_deactivate_) {
    run_stop_sequence(home_on_deactivate_, disable_on_deactivate_);
  } else {
    active_ = false;
  }
  return CallbackReturn::SUCCESS;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  run_stop_sequence(false, disable_on_shutdown_);
  bus_.close();
  return CallbackReturn::SUCCESS;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_shutdown(const rclcpp_lifecycle::State &) {
  if (home_on_shutdown_ || disable_on_shutdown_) {
    run_stop_sequence(home_on_shutdown_, disable_on_shutdown_);
  } else {
    active_ = false;
  }
  bus_.close();
  return CallbackReturn::SUCCESS;
}

ArmSystemHardware::CallbackReturn
ArmSystemHardware::on_error(const rclcpp_lifecycle::State &) {
  run_stop_sequence(false, true);
  bus_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ArmSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &period) {
  if (!bus_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  if (read_feedback_in_read_) {
    for (const auto &joint_config : joint_configs_) {
      read_motor_feedback(joint_config.motor_id);
    }
  } else {
    dm_j4340_motor_driver::UsbCanFrame frame;
    while (bus_.read_frame(frame, 0)) {
      if (dm_j4340_motor_driver::parse_register_reply(frame).has_value()) {
        continue;
      }
      const auto feedback = dm_j4340_motor_driver::parse_feedback(frame);
      if (feedback.has_value()) {
        latest_feedback_[feedback->motor_id] = feedback.value();
      }
    }
  }

  const double period_s = period.seconds();
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto feedback = latest_feedback_.find(joint_configs_[i].motor_id);
    if (feedback == latest_feedback_.end()) {
      continue;
    }

    const double new_position =
        motor_to_joint_position(i, feedback->second.position);
    hw_velocities_[i] = motor_to_joint_velocity(i, feedback->second.velocity);
    if (std::abs(hw_velocities_[i]) < 1e-6 && period_s > 1e-9) {
      hw_velocities_[i] = (new_position - previous_hw_positions_[i]) / period_s;
    }
    hw_positions_[i] = new_position;
    previous_hw_positions_[i] = new_position;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (!bus_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    if (!send_joint_position_command(i, hw_commands_[i])) {
      RCLCPP_ERROR(kLogger, "关节 %s 的电机控制帧发送失败。",
                   joint_names_[i].c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

bool ArmSystemHardware::send_frame(uint32_t can_id,
                                   const std::array<uint8_t, 8> &payload) {
  return bus_.send_frame(can_id, payload, 8, usbcan_command_, 1, 0, false,
                         false);
}

std::optional<dm_j4340_motor_driver::MotorFeedback>
ArmSystemHardware::read_motor_feedback(uint16_t motor_id) {
  for (int attempt = 0; attempt < retry_count_; ++attempt) {
    bus_.discard_pending();
    if (!send_frame(
            dm_j4340_motor_driver::kRegisterCanId,
            dm_j4340_motor_driver::make_refresh_feedback_command(motor_id))) {
      continue;
    }

    dm_j4340_motor_driver::UsbCanFrame frame;
    while (bus_.read_frame(frame, timeout_ms_)) {
      if (dm_j4340_motor_driver::parse_register_reply(frame).has_value()) {
        continue;
      }
      const auto feedback = dm_j4340_motor_driver::parse_feedback(frame);
      if (feedback.has_value() && feedback->motor_id == motor_id) {
        latest_feedback_[motor_id] = feedback.value();
        return feedback;
      }
    }
  }
  return std::nullopt;
}

std::optional<dm_j4340_motor_driver::RegisterReply>
ArmSystemHardware::read_motor_register(uint16_t motor_id, Register reg) {
  for (int attempt = 0; attempt < retry_count_; ++attempt) {
    bus_.discard_pending();
    if (!send_frame(
            dm_j4340_motor_driver::kRegisterCanId,
            dm_j4340_motor_driver::make_read_register_command(motor_id, reg))) {
      continue;
    }

    dm_j4340_motor_driver::UsbCanFrame frame;
    while (bus_.read_frame(frame, timeout_ms_)) {
      const auto reply = dm_j4340_motor_driver::parse_register_reply(frame);
      if (reply.has_value() && reply->motor_id == motor_id &&
          reply->reg == reg &&
          reply->command == dm_j4340_motor_driver::kRegisterReadCommand) {
        return reply;
      }
    }
  }
  return std::nullopt;
}

bool ArmSystemHardware::write_motor_register_uint32(uint16_t motor_id,
                                                    Register reg,
                                                    uint32_t value,
                                                    bool wait_reply) {
  const auto payload =
      dm_j4340_motor_driver::make_write_register_uint32_command(motor_id, reg,
                                                                value);
  for (int attempt = 0; attempt < retry_count_; ++attempt) {
    bus_.discard_pending();
    if (!send_frame(dm_j4340_motor_driver::kRegisterCanId, payload)) {
      continue;
    }
    if (!wait_reply) {
      return true;
    }

    dm_j4340_motor_driver::UsbCanFrame frame;
    while (bus_.read_frame(frame, timeout_ms_)) {
      const auto reply = dm_j4340_motor_driver::parse_register_reply(frame);
      if (reply.has_value() && reply->motor_id == motor_id &&
          reply->reg == reg &&
          reply->command == dm_j4340_motor_driver::kRegisterWriteCommand) {
        return true;
      }
    }
  }
  return false;
}

bool ArmSystemHardware::send_joint_position_command(size_t joint_index,
                                                    double joint_position) {
  if (joint_index >= joint_configs_.size()) {
    return false;
  }

  const auto &joint_config = joint_configs_[joint_index];
  double command = joint_position;
  if (joint_config.has_lower_limit) {
    command = std::max(command, joint_config.lower_limit);
  }
  if (joint_config.has_upper_limit) {
    command = std::min(command, joint_config.upper_limit);
  }

  const double motor_position = joint_to_motor_position(joint_index, command);
  const double motor_velocity_limit =
      std::abs(joint_to_motor_velocity(joint_index, joint_config.velocity_limit));

  switch (control_mode_) {
  case ControlMode::PositionVelocity:
    return send_frame(dm_j4340_motor_driver::control_can_id(
                          joint_config.motor_id, ControlMode::PositionVelocity),
                      dm_j4340_motor_driver::make_position_velocity_command(
                          static_cast<float>(motor_position),
                          static_cast<float>(motor_velocity_limit)));
  case ControlMode::Velocity:
    return send_frame(dm_j4340_motor_driver::control_can_id(
                          joint_config.motor_id, ControlMode::Velocity),
                      dm_j4340_motor_driver::make_velocity_command(0.0F));
  case ControlMode::ForcePosition:
    return send_frame(dm_j4340_motor_driver::control_can_id(
                          joint_config.motor_id, ControlMode::ForcePosition),
                      dm_j4340_motor_driver::make_force_position_command(
                          static_cast<float>(motor_position),
                          joint_config.force_position_velocity_limit,
                          joint_config.force_position_current_limit));
  case ControlMode::MIT:
  default:
    return send_frame(dm_j4340_motor_driver::control_can_id(
                          joint_config.motor_id, ControlMode::MIT),
                      dm_j4340_motor_driver::make_mit_command(
                          motor_position, 0.0, 30.0, 0.5, 0.0));
  }
}

void ArmSystemHardware::disable_motors() {
  if (!bus_.is_open()) {
    return;
  }

  for (int repeat = 0; repeat < disable_command_repeat_count_; ++repeat) {
    for (const auto &joint_config : joint_configs_) {
      if (!send_frame(joint_config.motor_id,
                      dm_j4340_motor_driver::make_disable_command())) {
        RCLCPP_WARN(kLogger, "电机 %u 失能命令发送失败。", joint_config.motor_id);
      }
    }
    if (repeat + 1 < disable_command_repeat_count_ &&
        disable_command_repeat_delay_ms_ > 0) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(disable_command_repeat_delay_ms_));
    }
  }

  RCLCPP_INFO(kLogger, "电机失能命令已发送 %d 轮。",
              disable_command_repeat_count_);
}

void ArmSystemHardware::move_to_zero_position() {
  if (!bus_.is_open() || joint_configs_.empty() ||
      shutdown_home_duration_s_ <= 0.0) {
    return;
  }

  const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, shutdown_home_command_rate_hz_));
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(shutdown_home_duration_s_);

  RCLCPP_INFO(kLogger, "停机回零开始：目标关节位置全 0，持续 %.2f s。",
              shutdown_home_duration_s_);

  while (std::chrono::steady_clock::now() < deadline) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      if (!send_joint_position_command(i, 0.0)) {
        RCLCPP_WARN(kLogger, "停机回零时关节 %s 命令发送失败。",
                    joint_names_[i].c_str());
      }
      hw_commands_[i] = 0.0;
    }
    std::this_thread::sleep_for(period);
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    send_joint_position_command(i, 0.0);
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(kLogger, "停机回零命令发送完成。");
}

void ArmSystemHardware::run_stop_sequence(bool home_first,
                                          bool disable_after) {
  if (!bus_.is_open()) {
    active_ = false;
    return;
  }
  if (stop_sequence_done_) {
    if (disable_after) {
      disable_motors();
    }
    active_ = false;
    return;
  }

  active_ = false;
  if (home_first) {
    move_to_zero_position();
  }
  if (disable_after) {
    disable_motors();
  }

  stop_sequence_done_ = true;
}

double ArmSystemHardware::joint_to_motor_position(size_t joint_index,
                                                  double joint_position) const {
  const auto &config = joint_configs_[joint_index];
  return config.zero_offset +
         config.direction * config.gear_ratio * joint_position;
}

double ArmSystemHardware::motor_to_joint_position(size_t joint_index,
                                                  double motor_position) const {
  const auto &config = joint_configs_[joint_index];
  return (motor_position - config.zero_offset) /
         (config.direction * config.gear_ratio);
}

double ArmSystemHardware::joint_to_motor_velocity(size_t joint_index,
                                                  double joint_velocity) const {
  const auto &config = joint_configs_[joint_index];
  return config.direction * config.gear_ratio * joint_velocity;
}

double ArmSystemHardware::motor_to_joint_velocity(size_t joint_index,
                                                  double motor_velocity) const {
  const auto &config = joint_configs_[joint_index];
  return motor_velocity / (config.direction * config.gear_ratio);
}

} // namespace arm_control

PLUGINLIB_EXPORT_CLASS(arm_control::ArmSystemHardware,
                       hardware_interface::SystemInterface)
