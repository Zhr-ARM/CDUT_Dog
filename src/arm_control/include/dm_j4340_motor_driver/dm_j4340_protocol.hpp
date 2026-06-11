#ifndef DM_J4340_MOTOR_DRIVER__DM_J4340_PROTOCOL_HPP_
#define DM_J4340_MOTOR_DRIVER__DM_J4340_PROTOCOL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

#include "dm_j4340_motor_driver/usbcan_serial.hpp"

namespace dm_j4340_motor_driver {

constexpr uint32_t kRegisterCanId = 0x7FF;
constexpr uint8_t kRegisterReadCommand = 0x33;
constexpr uint8_t kRegisterWriteCommand = 0x55;
constexpr uint8_t kRegisterStoreCommand = 0xAA;
constexpr uint32_t kPositionVelocityCanIdOffset = 0x100;
constexpr uint32_t kVelocityCanIdOffset = 0x200;
constexpr uint32_t kForcePositionCanIdOffset = 0x300;

/**
 * @brief DM-J4340 参数协议中使用的寄存器编号。
 */
enum class Register : uint8_t {
  UV_Value = 0x00,
  KT_Value = 0x01,
  OT_Value = 0x02,
  OC_Value = 0x03,
  ACC = 0x04,
  DEC = 0x05,
  MAX_SPD = 0x06,
  MST_ID = 0x07,
  ESC_ID = 0x08,
  TIMEOUT = 0x09,
  CTRL_MODE = 0x0A,
  Damp = 0x0B,
  Inertia = 0x0C,
  hw_ver = 0x0D,
  sw_ver = 0x0E,
  SN = 0x0F,
  NPP = 0x10,
  Rs = 0x11,
  Ls = 0x12,
  Flux = 0x13,
  Gr = 0x14,
  PMAX = 0x15,
  VMAX = 0x16,
  TMAX = 0x17,
  I_BW = 0x18,
  KP_ASR = 0x19,
  KI_ASR = 0x1A,
  KP_APR = 0x1B,
  KI_APR = 0x1C,
  OV_Value = 0x1D,
  GREF = 0x1E,
  Deta = 0x1F,
  V_BW = 0x20,
  IQ_c1 = 0x21,
  VL_c1 = 0x22,
  can_br = 0x23,
  sub_ver = 0x24,
  u_off = 0x32,
  v_off = 0x33,
  k1 = 0x34,
  k2 = 0x35,
  m_off = 0x36,
  dir = 0x37,
  p_m = 0x50,
  xout = 0x51,
};

/**
 * @brief 写入 CTRL_MODE 寄存器的控制模式编号。
 */
enum class ControlMode : uint32_t {
  MIT = 1,
  PositionVelocity = 2,
  Velocity = 3,
  ForcePosition = 4,
};

/**
 * @brief MIT 控制帧打包和反馈帧解算使用的量化范围。
 *
 * 默认值按 DM-J4340 设置。如果电机内部限制参数被修改，需要用实际参数覆盖。
 */
struct QuantizationLimits {
  double p_min = -12.5;
  double p_max = 12.5;
  double v_min = -8.0;
  double v_max = 8.0;
  double kp_min = 0.0;
  double kp_max = 500.0;
  double kd_min = 0.0;
  double kd_max = 5.0;
  double torque_min = -28.0;
  double torque_max = 28.0;
};

/**
 * @brief DM-J4340 状态反馈帧解算结果。
 */
struct MotorFeedback {
  uint16_t motor_id = 0;
  uint8_t error_code = 0;
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  uint8_t mos_temperature = 0;
  uint8_t rotor_temperature = 0;
};

/**
 * @brief 寄存器读写应答帧解算结果。
 */
struct RegisterReply {
  uint16_t motor_id = 0;
  uint8_t command = 0;
  Register reg = Register::UV_Value;
  uint32_t raw_value = 0;
  double numeric_value = 0.0;
  bool is_float = false;
};

/**
 * @brief 构造达妙特殊命令数据区，例如使能、失能或保存零点。
 */
inline std::array<uint8_t, 8> make_special_command(uint8_t suffix) {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, suffix};
}

/**
 * @brief 构造电机使能命令数据区，发送时 CAN ID 使用电机 ID。
 */
inline std::array<uint8_t, 8> make_enable_command() {
  return make_special_command(0xFC);
}

/**
 * @brief 构造电机失能命令数据区，发送时 CAN ID 使用电机 ID。
 */
inline std::array<uint8_t, 8> make_disable_command() {
  return make_special_command(0xFD);
}

/**
 * @brief 构造保存当前位置为零点的命令数据区。
 */
inline std::array<uint8_t, 8> make_save_zero_command() {
  return make_special_command(0xFE);
}

/**
 * @brief 构造清除电机错误的命令数据区。
 */
inline std::array<uint8_t, 8> make_clear_error_command() {
  return make_special_command(0xFB);
}

/**
 * @brief 构造寄存器读取请求数据区，发送时 CAN ID 使用 0x7FF。
 */
inline std::array<uint8_t, 8> make_read_register_command(uint16_t motor_id,
                                                         Register reg) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          kRegisterReadCommand,
          static_cast<uint8_t>(reg),
          0x00,
          0x00,
          0x00,
          0x00};
}

/**
 * @brief 构造刷新电机状态反馈的数据区，发送时 CAN ID 使用 0x7FF。
 */
inline std::array<uint8_t, 8> make_refresh_feedback_command(uint16_t motor_id) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          0xCC,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00};
}

/**
 * @brief 构造参数保存到 Flash 的请求数据区，发送时 CAN ID 使用 0x7FF。
 */
inline std::array<uint8_t, 8> make_store_parameters_command(uint16_t motor_id) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          kRegisterStoreCommand,
          0x01,
          0x00,
          0x00,
          0x00,
          0x00};
}

/**
 * @brief 构造 uint32 类型寄存器写入请求数据区，发送时 CAN ID 使用 0x7FF。
 */
inline std::array<uint8_t, 8>
make_write_register_uint32_command(uint16_t motor_id, Register reg,
                                   uint32_t value) {
  return {static_cast<uint8_t>(motor_id & 0xFF),
          static_cast<uint8_t>((motor_id >> 8) & 0xFF),
          kRegisterWriteCommand,
          static_cast<uint8_t>(reg),
          static_cast<uint8_t>(value & 0xFF),
          static_cast<uint8_t>((value >> 8) & 0xFF),
          static_cast<uint8_t>((value >> 16) & 0xFF),
          static_cast<uint8_t>((value >> 24) & 0xFF)};
}

/**
 * @brief 构造 float 类型寄存器写入请求数据区，发送时 CAN ID 使用 0x7FF。
 */
inline std::array<uint8_t, 8>
make_write_register_float_command(uint16_t motor_id, Register reg,
                                  float value) {
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  return make_write_register_uint32_command(motor_id, reg, raw);
}

/**
 * @brief 构造 CTRL_MODE 寄存器写入请求。
 */
inline std::array<uint8_t, 8> make_switch_mode_command(uint16_t motor_id,
                                                       ControlMode mode) {
  return make_write_register_uint32_command(motor_id, Register::CTRL_MODE,
                                            static_cast<uint32_t>(mode));
}

/**
 * @brief 根据控制模式返回控制帧应使用的 CAN ID。
 */
inline uint32_t control_can_id(uint16_t motor_id, ControlMode mode) {
  switch (mode) {
  case ControlMode::MIT:
    return motor_id;
  case ControlMode::PositionVelocity:
    return static_cast<uint32_t>(motor_id) + kPositionVelocityCanIdOffset;
  case ControlMode::Velocity:
    return static_cast<uint32_t>(motor_id) + kVelocityCanIdOffset;
  case ControlMode::ForcePosition:
    return static_cast<uint32_t>(motor_id) + kForcePositionCanIdOffset;
  default:
    return motor_id;
  }
}

/**
 * @brief 将浮点值量化为无符号定点字段。
 */
inline uint32_t float_to_uint(double value, double min_value, double max_value,
                              uint8_t bits) {
  const double clamped = std::min(std::max(value, min_value), max_value);
  const double span = max_value - min_value;
  const uint32_t max_int = (1u << bits) - 1u;
  return static_cast<uint32_t>(
      std::lround((clamped - min_value) * max_int / span));
}

/**
 * @brief 将无符号定点字段还原为浮点值。
 */
inline double uint_to_float(uint32_t value, double min_value, double max_value,
                            uint8_t bits) {
  const uint32_t max_int = (1u << bits) - 1u;
  return static_cast<double>(value) * (max_value - min_value) / max_int +
         min_value;
}

/**
 * @brief 构造 MIT 模式控制帧数据区。
 *
 * 电机处于 MIT 模式时，发送该数据区使用 CAN ID = 电机 ID。
 */
inline std::array<uint8_t, 8>
make_mit_command(double position, double velocity, double kp, double kd,
                 double torque,
                 const QuantizationLimits &limits = QuantizationLimits{}) {
  const uint32_t p = float_to_uint(position, limits.p_min, limits.p_max, 16);
  const uint32_t v = float_to_uint(velocity, limits.v_min, limits.v_max, 12);
  const uint32_t kp_u = float_to_uint(kp, limits.kp_min, limits.kp_max, 12);
  const uint32_t kd_u = float_to_uint(kd, limits.kd_min, limits.kd_max, 12);
  const uint32_t t =
      float_to_uint(torque, limits.torque_min, limits.torque_max, 12);

  return {static_cast<uint8_t>((p >> 8) & 0xFF),
          static_cast<uint8_t>(p & 0xFF),
          static_cast<uint8_t>((v >> 4) & 0xFF),
          static_cast<uint8_t>(((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)),
          static_cast<uint8_t>(kp_u & 0xFF),
          static_cast<uint8_t>((kd_u >> 4) & 0xFF),
          static_cast<uint8_t>(((kd_u & 0x0F) << 4) | ((t >> 8) & 0x0F)),
          static_cast<uint8_t>(t & 0xFF)};
}

/**
 * @brief 构造位置速度模式控制帧数据区。
 *
 * 发送该数据区时使用 control_can_id(motor_id, ControlMode::PositionVelocity)。
 */
inline std::array<uint8_t, 8> make_position_velocity_command(float position,
                                                             float velocity) {
  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &position, sizeof(position));
  std::memcpy(data.data() + 4, &velocity, sizeof(velocity));
  return data;
}

/**
 * @brief 构造速度模式控制帧数据区。
 *
 * 发送该数据区时使用 control_can_id(motor_id, ControlMode::Velocity)。
 */
inline std::array<uint8_t, 8> make_velocity_command(float velocity) {
  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &velocity, sizeof(velocity));
  return data;
}

/**
 * @brief 构造力位混合模式控制帧数据区。
 *
 * 发送该数据区时使用 control_can_id(motor_id, ControlMode::ForcePosition)。
 */
inline std::array<uint8_t, 8>
make_force_position_command(float position, uint16_t velocity_limit,
                            uint16_t current_limit) {
  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &position, sizeof(position));
  data[4] = static_cast<uint8_t>(velocity_limit & 0xFF);
  data[5] = static_cast<uint8_t>((velocity_limit >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>(current_limit & 0xFF);
  data[7] = static_cast<uint8_t>((current_limit >> 8) & 0xFF);
  return data;
}

/**
 * @brief 判断寄存器值是否按整数存储，而不是 IEEE float。
 */
inline bool register_is_uint32(Register reg) {
  const auto rid = static_cast<uint8_t>(reg);
  return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) ||
         (rid >= 35 && rid <= 36);
}

/**
 * @brief 按寄存器定义的数据类型解码 32 位原始值。
 */
inline double raw_to_register_value(Register reg, uint32_t raw) {
  if (register_is_uint32(reg)) {
    return static_cast<double>(raw);
  }

  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return static_cast<double>(value);
}

/**
 * @brief 解析电机返回的寄存器应答帧。
 *
 * 有效寄存器应答使用适配器命令字 0x11，数据区命令字为 0x33、0x55 或 0xAA。
 */
inline std::optional<RegisterReply>
parse_register_reply(const UsbCanFrame &frame) {
  if (frame.command != 0x11 || frame.dlc < 8) {
    return std::nullopt;
  }
  if (frame.data[2] != kRegisterReadCommand &&
      frame.data[2] != kRegisterWriteCommand &&
      frame.data[2] != kRegisterStoreCommand) {
    return std::nullopt;
  }

  RegisterReply reply;
  reply.motor_id = static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
  reply.command = frame.data[2];
  reply.reg = static_cast<Register>(frame.data[3]);
  reply.raw_value = static_cast<uint32_t>(frame.data[4]) |
                    (static_cast<uint32_t>(frame.data[5]) << 8) |
                    (static_cast<uint32_t>(frame.data[6]) << 16) |
                    (static_cast<uint32_t>(frame.data[7]) << 24);
  reply.is_float = !register_is_uint32(reply.reg);
  reply.numeric_value = raw_to_register_value(reply.reg, reply.raw_value);
  return reply;
}

/**
 * @brief 解析普通电机状态反馈帧。
 */
inline std::optional<MotorFeedback>
parse_feedback(const UsbCanFrame &frame,
               const QuantizationLimits &limits = QuantizationLimits{}) {
  if (frame.command != 0x11 || frame.dlc < 8) {
    return std::nullopt;
  }

  const uint32_t pos_raw =
      (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[2];
  const uint32_t vel_raw =
      (static_cast<uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const uint32_t torque_raw =
      (static_cast<uint32_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];

  MotorFeedback feedback;
  feedback.motor_id = frame.can_id != 0
                          ? static_cast<uint16_t>(frame.can_id)
                          : static_cast<uint16_t>(frame.data[0] & 0x0F);
  feedback.error_code = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
  feedback.position = uint_to_float(pos_raw, limits.p_min, limits.p_max, 16);
  feedback.velocity = uint_to_float(vel_raw, limits.v_min, limits.v_max, 12);
  feedback.torque =
      uint_to_float(torque_raw, limits.torque_min, limits.torque_max, 12);
  feedback.mos_temperature = frame.data[6];
  feedback.rotor_temperature = frame.data[7];
  return feedback;
}

/**
 * @brief 返回寄存器编号对应的稳定文本名称。
 */
inline std::string register_name(Register reg) {
  switch (reg) {
  case Register::ACC:
    return "ACC";
  case Register::DEC:
    return "DEC";
  case Register::MAX_SPD:
    return "MAX_SPD";
  case Register::MST_ID:
    return "MST_ID";
  case Register::ESC_ID:
    return "ESC_ID";
  case Register::TIMEOUT:
    return "TIMEOUT";
  case Register::CTRL_MODE:
    return "CTRL_MODE";
  case Register::sw_ver:
    return "sw_ver";
  case Register::PMAX:
    return "PMAX";
  case Register::VMAX:
    return "VMAX";
  case Register::TMAX:
    return "TMAX";
  case Register::can_br:
    return "can_br";
  default:
    return "REG_0x" + [&]() {
      std::ostringstream ss;
      ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<int>(reg);
      return ss.str();
    }();
  }
}

/**
 * @brief 返回 CTRL_MODE 数值对应的稳定文本名称。
 */
inline std::string control_mode_name(uint32_t mode) {
  switch (mode) {
  case 1:
    return "MIT";
  case 2:
    return "position_velocity";
  case 3:
    return "velocity";
  case 4:
    return "force_position";
  default:
    return "unknown";
  }
}

/**
 * @brief 返回反馈错误码或状态码对应的稳定文本名称。
 */
inline std::string error_name(uint8_t error_code) {
  switch (error_code) {
  case 0x0:
    return "disabled";
  case 0x1:
    return "enabled";
  case 0x8:
    return "over_voltage";
  case 0x9:
    return "under_voltage";
  case 0xA:
    return "over_current";
  case 0xB:
    return "mos_over_temperature";
  case 0xC:
    return "motor_over_temperature";
  case 0xD:
    return "communication_lost";
  case 0xE:
    return "overload";
  default:
    return "unknown";
  }
}

/**
 * @brief 将寄存器解码结果格式化为 ROS 日志和话题输出文本。
 */
inline std::string format_register_value(const RegisterReply &reply) {
  std::ostringstream ss;
  if (reply.is_float) {
    ss << std::fixed << std::setprecision(4) << reply.numeric_value;
  } else {
    ss << reply.raw_value;
    if (reply.reg == Register::CTRL_MODE) {
      ss << " (" << control_mode_name(reply.raw_value) << ")";
    }
  }
  return ss.str();
}

} // namespace dm_j4340_motor_driver

#endif // DM_J4340_MOTOR_DRIVER__DM_J4340_PROTOCOL_HPP_ 头文件保护
