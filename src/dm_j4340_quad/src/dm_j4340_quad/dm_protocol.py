from dataclasses import dataclass
from typing import Optional


# DM 寄存器协议常量
# - 本项目使用 11 位标准 CAN ID。
# - 寄存器访问帧使用 0x7FF，且电机 ID 位于 payload[0:2]。
CAN_REGISTER_FRAME_ID = 0x7FF

# 寄存器协议命令码（payload[2]）。
# 0x33：读寄存器，0x55：写寄存器，0xAA：将当前 RAM 参数保存到 NVM。
REGISTER_READ_COMMAND = 0x33
REGISTER_WRITE_COMMAND = 0x55
REGISTER_STORE_COMMAND = 0xAA

# 控制模式寄存器地址。向该寄存器写入 CTRL_MODE_* 之一即可切换模式。
REGISTER_CTRL_MODE = 0x0A

# REGISTER_CTRL_MODE 对应的控制模式取值。
# 这些值由固件协议定义，必须与电机协议手册保持一致。
CTRL_MODE_MIT = 1
CTRL_MODE_POSITION_VELOCITY = 2
CTRL_MODE_VELOCITY = 3
CTRL_MODE_FORCE_POSITION = 4


# 反馈状态高 4 位（nibble）解码表。
# parse_feedback() 会从 payload[0] 的高 4 位提取 error_code。
# 0x0/0x1 表示状态位（disabled/enabled），>= 0x8 表示故障/告警状态。
ERROR_CODE_MAP = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mosfet_over_temperature",
    0xC: "motor_over_temperature",
    0xD: "communication_lost",
    0xE: "overload",
}


# 协议数据结构：
# - @dataclass 会自动生成 __init__、__repr__、__eq__ 等常用方法。
# - frozen=True 将实例设为只读，避免指令或反馈对象在运行期被意外修改。
@dataclass(frozen=True)
class MITCommand:
    """MIT 控制模式的目标指令。"""

    position: float  # 目标位置
    velocity: float  # 目标速度
    kp: float  # 位置环比例增益
    kd: float  # 速度环阻尼增益
    torque: float  # 力矩前馈


@dataclass(frozen=True)
class MITLimits:
    """MIT 指令量化/反量化的物理量边界。"""

    p_min: float = -12.5  # 位置下限
    p_max: float = 12.5  # 位置上限
    v_min: float = -30.0  # 速度下限
    v_max: float = 30.0  # 速度上限
    kp_min: float = 0.0  # Kp 下限
    kp_max: float = 500.0  # Kp 上限
    kd_min: float = 0.0  # Kd 下限
    kd_max: float = 5.0  # Kd 上限
    t_min: float = -10.0  # 力矩下限
    t_max: float = 10.0  # 力矩上限


@dataclass(frozen=True)
class MotorFeedback:
    """8 字节反馈帧解析后的结构化结果。"""

    motor_id: int  # 电机 ID（来自状态字节低 4 位）
    error_code: int  # 状态/故障码（来自状态字节高 4 位）
    error_name: str  # 状态/故障码对应的可读名称
    position: float  # 实际位置
    velocity: float  # 实际速度
    torque: float  # 实际力矩
    mos_temperature: int  # MOS 管温度
    rotor_temperature: int  # 转子温度


@dataclass(frozen=True)
class RegisterReply:
    """寄存器读写应答的统一数据结构。"""

    motor_id: int  # 应答来源电机 ID
    command: int  # 命令码（读/写/保存）
    register_id: int  # 寄存器地址
    value: int  # 32 位寄存器值（小端）


def make_special_command(suffix: int) -> bytes:
    """生成 DM 特殊控制命令帧。

    帧格式固定为 8 字节：前 7 字节均为 0xFF，
    最后 1 字节为命令后缀（suffix）。
    """
    return bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, suffix & 0xFF])


def make_disable_command() -> bytes:
    """生成电机失能命令帧。"""
    return make_special_command(0xFD)


def make_enable_command() -> bytes:
    """生成电机使能命令帧。"""
    return make_special_command(0xFC)


def make_set_zero_command() -> bytes:
    """生成当前位置清零命令帧。"""
    return make_special_command(0xFE)


def make_clear_fault_command() -> bytes:
    """生成清除故障状态命令帧。"""
    return make_special_command(0xFB)


def make_read_register_command(motor_id: int, register_id: int) -> bytes:
    """生成寄存器读命令。

    参数:
    - motor_id: 目标电机 ID（16 位）。
    - register_id: 目标寄存器地址（8 位）。

    返回:
    - 8 字节寄存器访问负载。
    """
    return _make_register_command(
        motor_id,
        REGISTER_READ_COMMAND,
        register_id,
        0,
    )


def make_write_register_uint32_command(
    motor_id: int,
    register_id: int,
    value: int,
) -> bytes:
    """生成 uint32 寄存器写命令。

    参数:
    - motor_id: 目标电机 ID（16 位）。
    - register_id: 目标寄存器地址（8 位）。
    - value: 待写入的 32 位无符号值。
    """
    return _make_register_command(
        motor_id,
        REGISTER_WRITE_COMMAND,
        register_id,
        value,
    )


def _make_register_command(
    motor_id: int, command: int, register_id: int, value: int
) -> bytes:
    """按协议打包通用寄存器访问负载。

    字节布局:
    - [0:2]: motor_id（小端，低字节在前）
    - [2]: command
    - [3]: register_id
    - [4:8]: value（uint32，小端）
    """
    value_bytes = int(value).to_bytes(4, "little", signed=False)
    return (
        bytes(
            [
                motor_id & 0xFF,
                (motor_id >> 8) & 0xFF,
                command & 0xFF,
                register_id & 0xFF,
            ]
        )
        + value_bytes
    )


def pack_mit_command(command: MITCommand, limits: MITLimits) -> bytes:
    """将 MIT 控制目标打包为 8 字节命令负载。

    先按 limits 将浮点量量化到无符号整数，再按协议位宽拼接。
    """
    p_int = float_to_uint(command.position, limits.p_min, limits.p_max, 16)
    v_int = float_to_uint(command.velocity, limits.v_min, limits.v_max, 12)
    kp_int = float_to_uint(command.kp, limits.kp_min, limits.kp_max, 12)
    kd_int = float_to_uint(command.kd, limits.kd_min, limits.kd_max, 12)
    t_int = float_to_uint(command.torque, limits.t_min, limits.t_max, 12)

    # 字节布局（高位在前）：
    # B0-B1: P[15:0]
    # B2:    V[11:4]
    # B3:    V[3:0] | Kp[11:8]
    # B4:    Kp[7:0]
    # B5:    Kd[11:4]
    # B6:    Kd[3:0] | T[11:8]
    # B7:    T[7:0]
    return bytes(
        [
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
            t_int & 0xFF,
        ]
    )


def parse_feedback(payload: bytes, limits: MITLimits = MITLimits()) -> MotorFeedback:
    """解析电机 8 字节反馈帧并转换为物理量。

    参数:
    - payload: 原始反馈负载，必须为 8 字节。
    - limits: 与打包一致的量化边界，用于反量化。
    """
    if len(payload) < 8:
        raise ValueError("DM-J4340 feedback payload must be 8 bytes.")

    # 状态字节 payload[0]: 高 4 位为 error_code，低 4 位为 motor_id。
    error_code = (payload[0] >> 4) & 0x0F
    motor_id = payload[0] & 0x0F

    # 复原按位拼接的原始整数值。
    pos_raw = (payload[1] << 8) | payload[2]
    vel_raw = (payload[3] << 4) | (payload[4] >> 4)
    torque_raw = ((payload[4] & 0x0F) << 8) | payload[5]

    return MotorFeedback(
        motor_id=motor_id,
        error_code=error_code,
        error_name=ERROR_CODE_MAP.get(error_code, f"unknown_0x{error_code:X}"),
        position=uint_to_float(pos_raw, limits.p_min, limits.p_max, 16),
        velocity=uint_to_float(vel_raw, limits.v_min, limits.v_max, 12),
        torque=uint_to_float(torque_raw, limits.t_min, limits.t_max, 12),
        mos_temperature=int(payload[6]),
        rotor_temperature=int(payload[7]),
    )


def parse_register_reply(payload: bytes) -> Optional[RegisterReply]:
    """解析寄存器读写应答帧。

    返回:
    - RegisterReply: 当 payload 为合法寄存器应答时返回结构化结果。
    - None: 长度不足或命令码不属于寄存器访问类型。
    """
    if len(payload) < 8:
        return None
    if payload[2] not in (
        REGISTER_READ_COMMAND,
        REGISTER_WRITE_COMMAND,
        REGISTER_STORE_COMMAND,
    ):
        return None

    return RegisterReply(
        motor_id=payload[0] | (payload[1] << 8),
        command=payload[2],
        register_id=payload[3],
        value=int.from_bytes(payload[4:8], "little", signed=False),
    )


def float_to_uint(value: float, min_value: float, max_value: float, bits: int) -> int:
    """将浮点物理量线性映射为无符号整数。

    映射前会先钳位到 [min_value, max_value]，
    再映射到 [0, 2^bits - 1] 区间并四舍五入。
    """
    clamped = min(max(value, min_value), max_value)
    span = max_value - min_value
    scale = (1 << bits) - 1
    return int(round((clamped - min_value) * scale / span))


def uint_to_float(value: int, min_value: float, max_value: float, bits: int) -> float:
    """将无符号整数按线性比例反映射为浮点物理量。"""
    scale = (1 << bits) - 1
    span = max_value - min_value
    return float(value) * span / scale + min_value
