"""DM-J4340 MIT 电机控制主节点。

通过 USB-CAN 串口与 DM-J4340 电机通信，执行上电启动序列后周期发送 MIT
控制指令，发布关节状态并提供使能/失能/清零/清故障服务。
"""

import threading
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from real_motor_controller.dm_protocol import (
    CAN_REGISTER_FRAME_ID, CTRL_MODE_MIT, MITCommand, MITLimits, MotorFeedback,
    REGISTER_CTRL_MODE, REGISTER_WRITE_COMMAND,
    make_clear_fault_command, make_disable_command, make_enable_command,
    make_set_zero_command, make_write_register_uint32_command,
    mit_command_expired, pack_mit_command, parse_feedback, parse_register_reply,
)
from real_motor_controller.ros_params import declare_parameters, load_parameters
from real_motor_controller.usbcan_serial import UsbCanFrame, UsbCanSerial

# ── 默认参数（可通过 launch / yaml 覆盖） ──────────────────────
START_PARAMS = {
    # 串口
    "serial_port": "/dev/ttyACM0",    "serial_baudrate": 921600,
    "can_bitrate_index": 0,           "set_can_bitrate_on_start": True,
    # 电机映射
    "motor_ids": [1, 2, 3, 4],
    "joint_names": ["base_yaw_joint", "shoulder_pitch_joint",
                    "elbow_pitch_joint", "wrist_pitch_joint"],
    # CAN 协议
    "master_id": 0,                   "accept_any_master_id": True,
    "usbcan_send_cmd": 0x01,
    # 启动时序
    "startup_delay_s": 0.2,           "per_motor_start_delay_s": 0.03,
    "zero_mit_burst_count": 20,       "zero_mit_burst_period_s": 0.005,
    # 话题
    "mit_command_topic": "/dm_j4340/mit_commands",
    "dm_joint_states_topic": "/dm_j4340/joint_states",
    "robot_joint_states_topic": "/joint_states",
    "status_topic": "/dm_j4340/status",
    "publish_robot_joint_states": True,
    # 超时 / 频率
    "mit_command_timeout_s": 0.5,     "feedback_timeout_s": 0.5,
    "command_rate_hz": 200.0,         "state_rate_hz": 50.0,
    "status_rate_hz": 10.0,
    # 启动行为
    "auto_set_mit_mode_on_start": True,  "verify_mode_write_on_start": True,
    "register_reply_timeout_s": 0.3,     "register_retry_attempts": 3,
    "auto_clear_fault_on_start": True,   "auto_set_zero_on_start": True,
    "auto_enable_on_start": True,        "require_feedback_before_enable": True,
    "startup_feedback_timeout_s": 1.0,   "disable_on_shutdown": True,
    # MIT 限幅
    "p_min": -12.5, "p_max": 12.5,   "v_min": -30.0, "v_max": 30.0,
    "kp_min": 0.0,  "kp_max": 500.0, "kd_min": 0.0,  "kd_max": 5.0,
    "t_min": -10.0, "t_max": 10.0,
}

# MITLimits 对应的字段名，用于从参数构建限幅对象。
_LIMIT_KEYS = ("p_min","p_max","v_min","v_max","kp_min","kp_max","kd_min","kd_max","t_min","t_max")

# 零输出 MIT 指令 — 失能或超时时使用。
_ZERO_MIT = MITCommand(0.0, 0.0, 0.0, 0.0, 0.0)


class StartMotorNode(Node):
    """四电机 MIT 控制主节点。

    通过 USB-CAN 串口与 DM-J4340 电机通信：
    1) 声明参数 → 校验 → 打开串口 → 执行启动序列
    2) 以 200 Hz 周期发送 MIT 指令
    3) 以 50 Hz 发布关节状态，10 Hz 发布状态摘要
    """

    # ── 参数属性（由 _load_parameters 动态注入） ──
    serial_port: str
    serial_baudrate: int
    can_bitrate_index: int
    set_can_bitrate_on_start: bool
    motor_ids: List[int]
    joint_names: List[str]
    master_id: int
    accept_any_master_id: bool
    usbcan_send_cmd: int
    startup_delay_s: float
    per_motor_start_delay_s: float
    zero_mit_burst_count: int
    zero_mit_burst_period_s: float
    mit_command_topic: str
    dm_joint_states_topic: str
    robot_joint_states_topic: str
    status_topic: str
    publish_robot_joint_states: bool
    mit_command_timeout_s: float
    auto_set_mit_mode_on_start: bool
    verify_mode_write_on_start: bool
    register_reply_timeout_s: float
    register_retry_attempts: int
    auto_clear_fault_on_start: bool
    auto_set_zero_on_start: bool
    auto_enable_on_start: bool
    require_feedback_before_enable: bool
    startup_feedback_timeout_s: float
    disable_on_shutdown: bool
    command_rate_hz: float
    state_rate_hz: float
    status_rate_hz: float
    feedback_timeout_s: float
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float
    t_min: float
    t_max: float

    # ── 初始化 ─────────────────────────────────────────────

    def __init__(self) -> None:
        super().__init__("real_motor_controller_node")

        # 1) 参数：声明 → 加载 → 校验
        declare_parameters(self, START_PARAMS)
        self._load_parameters()
        self._validate_config()

        # 2) 共享状态（定时器 / RX 线程并发访问，各锁独立避免争用）
        self._state_lock = threading.Lock()
        self._feedback_by_id: Dict[int, MotorFeedback] = {}
        self._feedback_time_by_id: Dict[int, float] = {}

        self._register_lock = threading.Lock()
        self._register_replies: Dict[Tuple[int, int, int], int] = {}
        self._pending_register_keys: Set[Tuple[int, int, int]] = set()

        self._command_lock = threading.Lock()
        self._mit_commands = [_ZERO_MIT] * len(self.motor_ids)
        self._last_mit_command_time: Optional[float] = None

        self._last_commanded_torque = [0.0] * len(self.motor_ids)
        self._last_published_position: List[float] = [0.0] * len(self.motor_ids)
        self.enabled = False

        # 3) ROS 接口：publisher / subscriber / service
        self.dm_state_publisher = self.create_publisher(JointState, self.dm_joint_states_topic, 10)
        self.robot_state_publisher = self.create_publisher(JointState, self.robot_joint_states_topic, 10)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Float32MultiArray, self.mit_command_topic, self._on_mit_command, 10)

        # 四个控制服务复用同一工厂
        for svc_name, payload_fn, flag, label in (
            ("enable_all",      make_enable_command,      True,  "Enabled"),
            ("disable_all",     make_disable_command,     False, "Disabled"),
            ("set_zero_all",    make_set_zero_command,     None,  "Zero set"),
            ("clear_faults_all", make_clear_fault_command, None,  "Faults cleared"),
        ):
            self.create_service(
                Trigger, f"/dm_j4340/{svc_name}",
                self._make_service_handler(payload_fn, flag, label),
            )

        # 4) USB-CAN 链路
        self.bus = UsbCanSerial(self.serial_port, self.serial_baudrate,
                                frame_callback=self._on_usbcan_frame)
        self.bus.open()
        self.get_logger().info(f"Opened USB-CAN {self.serial_port} @ {self.serial_baudrate}")

        if self.set_can_bitrate_on_start:
            self.bus.set_can_bitrate(self.can_bitrate_index)
            self.get_logger().info(f"Set CAN bitrate index {self.can_bitrate_index}")
            time.sleep(self.startup_delay_s)

        # 5) 启动序列
        self._startup_sequence()

        # 6) 周期定时器
        self.create_timer(1.0 / self.command_rate_hz, self._command_timer_callback)
        self.create_timer(1.0 / self.state_rate_hz, self._publish_joint_state)
        self.create_timer(1.0 / self.status_rate_hz, self._publish_status)

    def _load_parameters(self) -> None:
        self.__dict__.update(load_parameters(self, START_PARAMS))
        self.limits = MITLimits(**{k: getattr(self, k) for k in _LIMIT_KEYS})

    def _validate_config(self) -> None:
        if len(self.joint_names) != len(self.motor_ids):
            raise ValueError("joint_names length must match motor_ids length.")
        for name in ("command_rate_hz", "state_rate_hz", "status_rate_hz"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")
        for name in ("register_retry_attempts", "register_reply_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")
        for name in ("zero_mit_burst_count", "zero_mit_burst_period_s", "mit_command_timeout_s"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be >= 0")

    # ── 服务工厂 ───────────────────────────────────────────

    def _make_service_handler(self, payload_fn, flag, label):
        """生成 Trigger 服务回调：发送特殊命令 + 可选更新 enabled 标志。"""
        def handler(_req, resp):
            self._send_to_all(payload_fn())
            if flag is not None:
                self.enabled = flag
            resp.success = True
            resp.message = f"{label} all motors."
            return resp
        return handler

    # ── 生命周期 ───────────────────────────────────────────

    def destroy_node(self) -> None:
        if hasattr(self, "bus"):
            if getattr(self, "disable_on_shutdown", False):
                try:
                    self._send_to_all(make_disable_command())
                except Exception as exc:
                    self.get_logger().warning(f"Shutdown disable failed: {exc}")
            self.bus.close()
        super().destroy_node()

    # ── 启动序列 ───────────────────────────────────────────

    def _startup_sequence(self) -> None:
        # 先失能再配置，保证寄存器操作在安全状态
        self._send_to_all(make_disable_command())
        self.enabled = False
        self.get_logger().info("Disabled all motors before startup config.")

        if self.auto_set_mit_mode_on_start:
            if not self._set_mit_mode_all():
                self.get_logger().error("MIT mode not confirmed — aborting startup.")
                return
            self.get_logger().info("All motors confirmed CTRL_MODE=MIT.")

        if self.auto_clear_fault_on_start:
            self._send_to_all(make_clear_fault_command())
            self.get_logger().info("Clear-fault sent.")
        if self.auto_set_zero_on_start:
            self._send_to_all(make_set_zero_command())
            self.get_logger().info("Set-zero sent.")

        if self.auto_enable_on_start:
            if self.require_feedback_before_enable and not self._wait_feedback(
                self.startup_feedback_timeout_s
            ):
                missing = self._missing_feedback_ids()
                self.get_logger().error(f"Feedback missing from motors: {missing}")
                return
            # 发送零 MIT + 使能 + 零 MIT 突发以平滑切入闭环
            self._send_to_all(pack_mit_command(_ZERO_MIT, self.limits))
            self._send_to_all(make_enable_command())
            for _ in range(self.zero_mit_burst_count):
                self._send_to_all(pack_mit_command(_ZERO_MIT, self.limits))
                time.sleep(self.zero_mit_burst_period_s)
            self.enabled = True
            self.get_logger().info(f"Enabled MIT command mode on {self.mit_command_topic}")

    def _wait_feedback(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self._missing_feedback_ids():
                return True
            time.sleep(0.01)
        return not self._missing_feedback_ids()

    def _missing_feedback_ids(self) -> List[int]:
        with self._state_lock:
            return [mid for mid in self.motor_ids if mid not in self._feedback_by_id]

    def _set_mit_mode_all(self) -> bool:
        all_ok = True
        for mid in self.motor_ids:
            if self._write_register_uint32(mid, REGISTER_CTRL_MODE, CTRL_MODE_MIT):
                self.get_logger().info(f"Motor {mid}: CTRL_MODE=MIT OK")
            else:
                all_ok = False
                self.get_logger().error(f"Motor {mid}: CTRL_MODE=MIT FAILED")
            time.sleep(self.per_motor_start_delay_s)
        return all_ok

    # ── CAN 发送 ───────────────────────────────────────────

    def _send_to_all(self, payload: bytes) -> None:
        """向全部电机发送同一负载，保留电机间隔。"""
        for mid in self.motor_ids:
            self.bus.send_can_frame(mid, payload, command=0x01,
                                    send_times=1, send_interval_100us=0,
                                    is_extended=False, is_remote=False)
            time.sleep(self.per_motor_start_delay_s)

    # ── 寄存器写入 ─────────────────────────────────────────

    def _write_register_uint32(self, motor_id: int, register_id: int, value: int) -> bool:
        key = (motor_id, register_id, REGISTER_WRITE_COMMAND)
        payload = make_write_register_uint32_command(motor_id, register_id, value)
        with self._register_lock:
            self._register_replies.pop(key, None)
            self._pending_register_keys.add(key)

        for attempt in range(1, self.register_retry_attempts + 1):
            self.bus.send_can_frame(CAN_REGISTER_FRAME_ID, payload, command=0x01,
                                    send_times=1, send_interval_100us=0,
                                    is_extended=False, is_remote=False)
            if not self.verify_mode_write_on_start:
                with self._register_lock:
                    self._pending_register_keys.discard(key)
                return True
            if self._await_register_ack(key, value, self.register_reply_timeout_s):
                return True
            self.get_logger().warning(
                f"Motor {motor_id}: no ACK for reg 0x{register_id:02X} attempt {attempt}"
            )

        with self._register_lock:
            self._pending_register_keys.discard(key)
        return False

    def _await_register_ack(self, key: Tuple[int, int, int],
                            expected: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._register_lock:
                if self._register_replies.get(key) == expected:
                    return True
            time.sleep(0.005)
        return False

    # ── 周期回调 ───────────────────────────────────────────

    def _command_timer_callback(self) -> None:
        if not self.enabled:
            return
        commands = self._current_mit_commands()
        for mid, cmd in zip(self.motor_ids, commands):
            self.bus.send_can_frame(mid, pack_mit_command(cmd, self.limits), command=0x01,
                                    send_times=1, send_interval_100us=0,
                                    is_extended=False, is_remote=False)
        with self._state_lock:
            self._last_commanded_torque = [c.torque for c in commands]

    def _current_mit_commands(self) -> List[MITCommand]:
        with self._command_lock:
            if mit_command_expired(self._last_mit_command_time, self.mit_command_timeout_s):
                return [_ZERO_MIT] * len(self.motor_ids)
            return list(self._mit_commands)

    # ── 状态发布 ───────────────────────────────────────────

    def _publish_joint_state(self) -> None:
        """发布关节状态；某电机反馈丢失时沿用上一次位置，避免误判为零点。"""
        with self._state_lock:
            fb_list = [self._feedback_by_id.get(mid) for mid in self.motor_ids]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        pos, vel, eff = [], [], []
        for i, fb in enumerate(fb_list):
            if fb is not None:
                pos.append(fb.position); vel.append(fb.velocity); eff.append(fb.torque)
                self._last_published_position[i] = fb.position
            else:
                pos.append(self._last_published_position[i])
                vel.append(0.0); eff.append(0.0)
        msg.position = pos; msg.velocity = vel; msg.effort = eff
        self.dm_state_publisher.publish(msg)
        if self.publish_robot_joint_states:
            self.robot_state_publisher.publish(msg)

    def _publish_status(self) -> None:
        """发布单行状态摘要：使能态 + 模式 + 各电机在线/温度/延迟。"""
        now = time.monotonic()
        with self._command_lock:
            cmd_fresh = not mit_command_expired(self._last_mit_command_time, self.mit_command_timeout_s)
        mode = "disabled" if not self.enabled else ("topic_mit" if cmd_fresh else "zero_mit")

        with self._state_lock:
            fb_by_id = dict(self._feedback_by_id)
            fb_time = dict(self._feedback_time_by_id)

        parts = [f"enabled={int(self.enabled)} mode={mode}"]
        for mid in self.motor_ids:
            fb = fb_by_id.get(mid); t = fb_time.get(mid)
            if fb is None or t is None:
                parts.append(f"id{mid}=missing")
            else:
                age = now - t
                parts.append(
                    f"id{mid}={fb.error_name} fresh={int(age <= self.feedback_timeout_s)}"
                    f" age={age:.2f} mos={fb.mos_temperature}C rotor={fb.rotor_temperature}C"
                )
        text = " ".join(parts)
        self.status_publisher.publish(String(data=text))
        self.get_logger().debug(text)

    # ── USB-CAN 帧接收（RX 线程回调） ────────────────────────

    def _on_usbcan_frame(self, frame: UsbCanFrame) -> None:
        """处理回传帧：寄存器应答 → 反馈解析，均在锁保护下完成。"""
        if (frame.command != 0x11 or len(frame.payload) < 8
                or (not self.accept_any_master_id and frame.can_id != self.master_id)):
            return

        # 寄存器应答：在锁内 parse → 匹配 → 存值，避免 TOCTOU 竞态
        reply = parse_register_reply(frame.payload)
        if reply is not None:
            key = (reply.motor_id, reply.register_id, reply.command)
            with self._register_lock:
                if key in self._pending_register_keys:
                    self._register_replies[key] = reply.value
                    self._pending_register_keys.discard(key)
                    return

        # 反馈帧
        try:
            fb = parse_feedback(frame.payload, self.limits)
        except ValueError:
            return
        if fb.motor_id not in self.motor_ids:
            return
        with self._state_lock:
            self._feedback_by_id[fb.motor_id] = fb
            self._feedback_time_by_id[fb.motor_id] = time.monotonic()

    # ── MIT 指令订阅 ───────────────────────────────────────

    def _on_mit_command(self, msg: Float32MultiArray) -> None:
        """解析 MIT 指令：[p,v,kp,kd,t_ff] × motor_count。"""
        values = [float(v) for v in msg.data]
        n = len(self.motor_ids) * 5
        if len(values) != n:
            self.get_logger().warning(f"MIT cmd expects {n} values, got {len(values)}")
            return
        cmds = [MITCommand(values[i], values[i+1], values[i+2], values[i+3], values[i+4])
                for i in range(0, n, 5)]
        with self._command_lock:
            self._mit_commands = cmds
            self._last_mit_command_time = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StartMotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
