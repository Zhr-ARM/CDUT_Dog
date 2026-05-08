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


# ── 特殊命令（前 7 字节 0xFF，末字节为命令后缀） ────────────

def _special_cmd(suffix: int) -> bytes:
    return bytes([0xFF]*7 + [suffix & 0xFF])

_DISABLE_CMD    = _special_cmd(0xFD)
_ENABLE_CMD     = _special_cmd(0xFC)
_SET_ZERO_CMD   = _special_cmd(0xFE)
_CLEAR_FAULT_CMD = _special_cmd(0xFB)

def make_disable_command() -> bytes:      return _DISABLE_CMD
def make_enable_command() -> bytes:       return _ENABLE_CMD
def make_set_zero_command() -> bytes:     return _SET_ZERO_CMD
def make_clear_fault_command() -> bytes:   return _CLEAR_FAULT_CMD

# ── 寄存器读写 ──────────────────────────────────────────────

def make_write_register_uint32_command(motor_id: int, register_id: int, value: int) -> bytes:
    """打包 uint32 寄存器写命令。"""
    return _pack_register_cmd(motor_id, REGISTER_WRITE_COMMAND, register_id, value)


def _pack_register_cmd(
    motor_id: int, command: int, register_id: int, value: int
) -> bytes:
    """打包寄存器访问负载：[0:2]motor_id [2]cmd [3]reg [4:8]value(LE)。"""
    return bytes([
        motor_id & 0xFF, (motor_id >> 8) & 0xFF,
        command & 0xFF, register_id & 0xFF,
    ]) + int(value).to_bytes(4, "little", signed=False)


def pack_mit_command(command: MITCommand, limits: MITLimits) -> bytes:
    """将 MIT 指令量化为 8 字节 CAN 负载。

    位布局: P[16] V[12] Kp[12] Kd[12] T[12] → 按协议交错拼接。
    """
    p  = _f2u(command.position, limits.p_min, limits.p_max, 16)
    v  = _f2u(command.velocity, limits.v_min, limits.v_max, 12)
    kp = _f2u(command.kp,       limits.kp_min, limits.kp_max, 12)
    kd = _f2u(command.kd,       limits.kd_min, limits.kd_max, 12)
    t  = _f2u(command.torque,   limits.t_min, limits.t_max, 12)
    return bytes([
        (p >> 8) & 0xFF,  p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F),
        kp & 0xFF,
        (kd >> 4) & 0xFF,
        ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
    ])


def parse_feedback(payload: bytes, limits: MITLimits = MITLimits()) -> MotorFeedback:
    """解析 8 字节反馈帧 → MotorFeedback。

    payload[0]: high-nibble=error_code, low-nibble=motor_id
    payload[1:6]: 位拼接的位置/速度/力矩, [6]=MOS温度, [7]=转子温度
    """
    if len(payload) < 8:
        raise ValueError("Feedback payload must be 8 bytes.")
    ec = (payload[0] >> 4) & 0x0F
    return MotorFeedback(
        motor_id=payload[0] & 0x0F,
        error_code=ec,
        error_name=ERROR_CODE_MAP.get(ec, f"unknown_0x{ec:X}"),
        position=_u2f((payload[1] << 8) | payload[2], limits.p_min, limits.p_max, 16),
        velocity=_u2f((payload[3] << 4) | (payload[4] >> 4), limits.v_min, limits.v_max, 12),
        torque=_u2f(((payload[4] & 0x0F) << 8) | payload[5], limits.t_min, limits.t_max, 12),
        mos_temperature=int(payload[6]),
        rotor_temperature=int(payload[7]),
    )


def parse_register_reply(payload: bytes) -> Optional[RegisterReply]:
    """解析寄存器应答帧，非法时返回 None。"""
    if len(payload) < 8 or payload[2] not in (
        REGISTER_READ_COMMAND, REGISTER_WRITE_COMMAND, REGISTER_STORE_COMMAND,
    ):
        return None
    return RegisterReply(
        motor_id=payload[0] | (payload[1] << 8),
        command=payload[2],
        register_id=payload[3],
        value=int.from_bytes(payload[4:8], "little", signed=False),
    )


# ── 量化 / 反量化 ──────────────────────────────────────────

def _f2u(value: float, lo: float, hi: float, bits: int) -> int:
    """浮点 → 无符号整数：钳位 → 线性映射 → 四舍五入。"""
    v = min(max(value, lo), hi)
    return int(round((v - lo) * ((1 << bits) - 1) / (hi - lo)))


def _u2f(value: int, lo: float, hi: float, bits: int) -> float:
    """无符号整数 → 浮点：线性反映射。"""
    return float(value) * (hi - lo) / ((1 << bits) - 1) + lo


# ── 共享超时判断（实机 / 仿真共用） ──────────────────────────

import time as _time  # noqa: E402


def mit_command_expired(last_time: float | None, timeout_s: float) -> bool:
    """MIT 指令超时判定：从未收到或距上次超过 timeout_s 则返回 True。"""
    if last_time is None:
        return True
    if timeout_s <= 0.0:
        return False
    return _time.monotonic() - last_time > timeout_s
