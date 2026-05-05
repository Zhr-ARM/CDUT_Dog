"""DM-J4340 启动与控制节点。

职责:
- 执行上电启动序列（模式设置、清故障、清零、使能）。
- 周期发送 MIT 控制指令。
- 发布关节状态与文本状态，并提供常用控制服务。
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
    CAN_REGISTER_FRAME_ID,
    CTRL_MODE_MIT,
    MITCommand,
    MITLimits,
    MotorFeedback,
    REGISTER_CTRL_MODE,
    REGISTER_WRITE_COMMAND,
    make_clear_fault_command,
    make_disable_command,
    make_enable_command,
    make_set_zero_command,
    make_write_register_uint32_command,
    mit_command_expired,
    pack_mit_command,
    parse_feedback,
    parse_register_reply,
)
from real_motor_controller.ros_params import declare_parameters, load_parameters
from real_motor_controller.usbcan_serial import UsbCanFrame, UsbCanSerial


# 启动节点参数默认值。
START_PARAMS = {
    "serial_port": "/dev/ttyACM0",
    "serial_baudrate": 921600,
    "can_bitrate_index": 0,
    "set_can_bitrate_on_start": True,
    "motor_ids": [1, 2, 3, 4],
    "joint_names": [
        "base_yaw_joint",
        "shoulder_pitch_joint",
        "elbow_pitch_joint",
        "wrist_pitch_joint",
    ],
    "master_id": 0,
    "accept_any_master_id": True,
    "usbcan_send_cmd": 0x01,
    "startup_delay_s": 0.2,
    "per_motor_start_delay_s": 0.03,
    "zero_mit_burst_count": 20,
    "zero_mit_burst_period_s": 0.005,
    "mit_command_topic": "/dm_j4340/mit_commands",
    "dm_joint_states_topic": "/dm_j4340/joint_states",
    "robot_joint_states_topic": "/joint_states",
    "status_topic": "/dm_j4340/status",
    "publish_robot_joint_states": True,
    "mit_command_timeout_s": 0.2,
    "auto_set_mit_mode_on_start": True,
    "verify_mode_write_on_start": True,
    "register_reply_timeout_s": 0.3,
    "register_retry_attempts": 3,
    "auto_clear_fault_on_start": True,
    "auto_set_zero_on_start": True,
    "auto_enable_on_start": True,
    "require_feedback_before_enable": True,
    "startup_feedback_timeout_s": 1.0,
    "disable_on_shutdown": True,
    "command_rate_hz": 200.0,
    "state_rate_hz": 50.0,
    "status_rate_hz": 10.0,
    "feedback_timeout_s": 0.5,
    "p_min": -12.5,
    "p_max": 12.5,
    "v_min": -30.0,
    "v_max": 30.0,
    "kp_min": 0.0,
    "kp_max": 500.0,
    "kd_min": 0.0,
    "kd_max": 5.0,
    "t_min": -10.0,
    "t_max": 10.0,
}

# MIT 量化边界字段名，用于构建 MITLimits。
LIMIT_FIELDS = (
    "p_min",
    "p_max",
    "v_min",
    "v_max",
    "kp_min",
    "kp_max",
    "kd_min",
    "kd_max",
    "t_min",
    "t_max",
)

# 需要满足非负约束的参数集合。
NON_NEGATIVE_PARAMS = (
    "zero_mit_burst_count",
    "zero_mit_burst_period_s",
    "mit_command_timeout_s",
)

# USB-CAN 发送配置：单次发送、标准帧、数据帧。
SEND_ONCE = {
    "send_times": 1,
    "send_interval_100us": 0,
    "is_extended": False,
    "is_remote": False,
}

# 零输出 MIT 指令，启动期间用于平滑进入闭环。
ZERO_MIT_COMMAND = MITCommand(0.0, 0.0, 0.0, 0.0, 0.0)


class StartMotorNode(Node):
    """四电机 MIT 控制主节点。"""

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

    def __init__(self) -> None:
        """初始化节点并完成电机控制主循环的启动前准备。

        初始化阶段说明:
        1) 节点基础构造（注册 ROS 节点名）。
        2) 参数声明、加载与配置校验。
        3) 初始化并发共享状态（反馈缓存、寄存器应答缓存、启动状态）。
        4) 创建话题/服务接口。
        5) 建立 USB-CAN 链路并按需设置波特率。
        6) 执行电机启动序列（模式设置、清故障、清零、使能）。
        7) 创建控制与状态定时器，进入周期运行。
        """
        super().__init__("real_motor_controller_node")

        # 参数初始化顺序必须是“声明 -> 读取 -> 校验”。
        # 这样既能支持 launch 覆盖参数，也能在启动期尽早发现非法配置。
        declare_parameters(self, START_PARAMS)
        self._load_parameters()
        self._validate_config()

        # -------- 运行时共享状态（由定时器回调与串口回调并发访问）--------
        # 电机反馈数据与时间戳缓存。
        self._state_lock = threading.Lock()
        self._feedback_by_id: Dict[int, MotorFeedback] = {}
        self._feedback_time_by_id: Dict[int, float] = {}

        # 寄存器写入 ACK 跟踪。
        # key 结构为 (motor_id, register_id, command)。
        self._register_lock = threading.Lock()
        self._register_replies: Dict[Tuple[int, int, int], int] = {}
        self._pending_register_keys: Set[Tuple[int, int, int]] = set()

        # 上层 MIT 指令缓存。启动后未收到指令或指令超时时保持零 MIT。
        self._command_lock = threading.Lock()
        self._mit_commands = [ZERO_MIT_COMMAND] * len(self.motor_ids)
        self._last_mit_command_time: Optional[float] = None

        self._last_commanded_torque = [0.0] * len(self.motor_ids)

        # MIT 命令发送总开关。False 时命令定时器直接返回。
        self.enabled = False

        # -------- ROS 通信接口 --------
        # 关节状态发布：位置/速度/力矩。
        self.dm_state_publisher = self.create_publisher(
            JointState, self.dm_joint_states_topic, 10
        )
        self.robot_state_publisher = self.create_publisher(
            JointState, self.robot_joint_states_topic, 10
        )

        # 设备状态摘要发布：在线性、温度、故障状态等。
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)

        # 外部 MIT 控制指令订阅。
        self.mit_command_subscriber = self.create_subscription(
            Float32MultiArray,
            self.mit_command_topic,
            self._handle_mit_command,
            10,
        )

        # 常用控制服务：使能/失能/清零/清故障。
        self.enable_service = self.create_service(
            Trigger, "/dm_j4340/enable_all", self._handle_enable_all
        )
        self.disable_service = self.create_service(
            Trigger, "/dm_j4340/disable_all", self._handle_disable_all
        )
        self.zero_service = self.create_service(
            Trigger, "/dm_j4340/set_zero_all", self._handle_set_zero_all
        )
        self.clear_fault_service = self.create_service(
            Trigger, "/dm_j4340/clear_faults_all", self._handle_clear_faults_all
        )

        # -------- USB-CAN 链路初始化 --------
        # frame_callback 在接收线程中被调用，用于处理反馈与寄存器应答。
        self.bus = UsbCanSerial(
            self.serial_port,
            self.serial_baudrate,
            frame_callback=self._handle_usbcan_frame,
        )
        self.bus.open()
        self.get_logger().info(
            f"Opened USB-CAN serial port {self.serial_port} at {self.serial_baudrate}."
        )

        # 部分适配器需要显式下发波特率设置；发送后等待设备生效。
        if self.set_can_bitrate_on_start:
            self.bus.set_can_bitrate(self.can_bitrate_index)
            self.get_logger().info(
                f"Sent USB-CAN bitrate index {self.can_bitrate_index} command."
            )
            time.sleep(self.startup_delay_s)

        # 启动序列会按配置执行模式设置、清故障、清零、使能等步骤。
        self._startup_sequence()

        # -------- 周期任务定时器 --------
        # 控制定时器：周期发送 MIT 命令。
        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz, self._command_timer_callback
        )

        # 状态发布定时器：发布 JointState。
        self.state_timer = self.create_timer(
            1.0 / self.state_rate_hz, self._publish_joint_state
        )

        # 文本状态发布定时器：发布状态摘要字符串。
        self.status_timer = self.create_timer(
            1.0 / self.status_rate_hz, self._publish_status
        )

    def _load_parameters(self) -> None:
        """读取参数并构建运行期派生配置。"""
        self.__dict__.update(load_parameters(self, START_PARAMS))
        self.limits = MITLimits(
            **{name: getattr(self, name) for name in LIMIT_FIELDS}
        )

    def _validate_config(self) -> None:
        """校验参数长度与数值范围，异常时直接抛错终止启动。"""
        for name, values in (
            ("joint_names", self.joint_names),
        ):
            if len(values) != len(self.motor_ids):
                raise ValueError(f"{name} length must match motor_ids length.")

        if any(
            value <= 0
            for value in (self.command_rate_hz, self.state_rate_hz, self.status_rate_hz)
        ):
            raise ValueError("Timer rates must be greater than zero.")

        for name in ("register_retry_attempts", "register_reply_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be greater than zero.")
        for name in NON_NEGATIVE_PARAMS:
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be zero or greater.")

    def destroy_node(self) -> None:
        """节点销毁钩子：按需失能电机并关闭串口。"""
        if hasattr(self, "bus"):
            if getattr(self, "disable_on_shutdown", False):
                try:
                    self._send_special_to_all(make_disable_command())
                except Exception as exc:
                    self.get_logger().warning(f"Failed to disable motors on shutdown: {exc}")
            self.bus.close()
        super().destroy_node()

    def _startup_sequence(self) -> None:
        """执行启动序列并在成功时进入使能状态。"""
        # 先失能，保证后续寄存器配置和清零过程在安全状态下执行。
        self._send_special_to_all(make_disable_command())
        self.enabled = False
        self.get_logger().info("Sent disable command before startup configuration.")

        if self.auto_set_mit_mode_on_start:
            if not self._set_mit_mode_all():
                self.enabled = False
                self.get_logger().error(
                    "Not enabling motors because at least one motor did not confirm MIT mode."
                )
                return
            self.get_logger().info("Confirmed CTRL_MODE=MIT for all motors.")

        if self.auto_clear_fault_on_start:
            self._send_special_to_all(make_clear_fault_command())
            self.get_logger().info("Sent clear-fault command to all motors.")

        if self.auto_set_zero_on_start:
            self._send_special_to_all(make_set_zero_command())
            self.get_logger().info("Sent set-zero command to all motors.")

        if self.auto_enable_on_start:
            if self.require_feedback_before_enable and not self._wait_for_all_feedback(
                self.startup_feedback_timeout_s
            ):
                missing = self._missing_feedback_ids()
                self.enabled = False
                self.get_logger().error(
                    f"Not enabling MIT commands because feedback is missing from motors: {missing}"
                )
                return
            self._send_zero_mit_to_all()
            self._send_special_to_all(make_enable_command())
            # 使能后短时间重复发送零 MIT，帮助控制器稳定进入闭环。
            self._send_zero_mit_burst()
            self.enabled = True
            self.get_logger().info(
                f"Started MIT topic-command mode, listening on {self.mit_command_topic}."
            )
            self.get_logger().info("Sent enable command to all motors.")

    def _wait_for_all_feedback(self, timeout_s: float) -> bool:
        """在超时窗口内等待所有目标电机都收到反馈。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self._missing_feedback_ids():
                return True
            time.sleep(0.01)
        return not self._missing_feedback_ids()

    def _missing_feedback_ids(self) -> List[int]:
        """返回当前尚未收到反馈的电机 ID 列表。"""
        with self._state_lock:
            return [
                motor_id
                for motor_id in self.motor_ids
                if motor_id not in self._feedback_by_id
            ]

    def _send_frame(self, can_id: int, payload: bytes, command: Optional[int] = None) -> None:
        """统一 CAN 发送入口，默认使用参数指定的 USB-CAN 命令字。"""
        self.bus.send_can_frame(
            can_id,
            payload,
            command=self.usbcan_send_cmd if command is None else command,
            **SEND_ONCE,
        )

    def _send_special_to_all(self, payload: bytes) -> None:
        """向全部电机顺序发送特殊命令，并保留电机间隔。"""
        for motor_id in self.motor_ids:
            self._send_frame(motor_id, payload, command=0x01)
            time.sleep(self.per_motor_start_delay_s)

    def _send_zero_mit_burst(self) -> None:
        """按配置次数发送零 MIT 突发指令。"""
        for _ in range(self.zero_mit_burst_count):
            self._send_zero_mit_to_all()
            time.sleep(self.zero_mit_burst_period_s)

    def _send_zero_mit_to_all(self) -> None:
        """向全部电机发送零输出 MIT 指令。"""
        payload = pack_mit_command(ZERO_MIT_COMMAND, self.limits)
        for motor_id in self.motor_ids:
            self._send_frame(motor_id, payload)

    def _set_mit_mode_all(self) -> bool:
        """将全部电机控制模式写为 MIT，并验证写入结果。"""
        all_ok = True
        for motor_id in self.motor_ids:
            ok = self._write_register_uint32(
                motor_id,
                REGISTER_CTRL_MODE,
                CTRL_MODE_MIT,
            )
            if ok:
                self.get_logger().info(f"Motor ID {motor_id}: CTRL_MODE set to MIT.")
            else:
                all_ok = False
                self.get_logger().error(
                    f"Motor ID {motor_id}: failed to confirm CTRL_MODE=MIT."
                )
            time.sleep(self.per_motor_start_delay_s)
        return all_ok

    def _write_register_uint32(
        self,
        motor_id: int,
        register_id: int,
        value: int,
    ) -> bool:
        """写入 32 位寄存器并按配置进行 ACK 重试确认。"""
        key = (motor_id, register_id, REGISTER_WRITE_COMMAND)
        payload = make_write_register_uint32_command(motor_id, register_id, value)
        with self._register_lock:
            self._register_replies.pop(key, None)
            self._pending_register_keys.add(key)

        for attempt in range(1, self.register_retry_attempts + 1):
            self._send_frame(CAN_REGISTER_FRAME_ID, payload, command=0x01)

            if not self.verify_mode_write_on_start:
                with self._register_lock:
                    self._pending_register_keys.discard(key)
                return True

            if self._wait_for_register_reply(key, value, self.register_reply_timeout_s):
                return True

            self.get_logger().warning(
                f"Motor ID {motor_id}: no register ACK for 0x{register_id:02X} "
                f"on attempt {attempt}."
            )

        with self._register_lock:
            self._pending_register_keys.discard(key)
        return False

    def _wait_for_register_reply(
        self,
        key: Tuple[int, int, int],
        expected_value: int,
        timeout_s: float,
    ) -> bool:
        """等待指定寄存器应答值出现。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._register_lock:
                value = self._register_replies.get(key)
            if value == expected_value:
                return True
            time.sleep(0.005)
        return False

    def _command_timer_callback(self) -> None:
        """控制周期回调：向全部电机下发当前有效 MIT 指令。"""
        if not self.enabled:
            return

        commands = self._current_mit_commands()
        for motor_id, command in zip(self.motor_ids, commands):
            self._send_frame(motor_id, pack_mit_command(command, self.limits))

        with self._state_lock:
            self._last_commanded_torque = [command.torque for command in commands]

    def _current_mit_commands(self) -> List[MITCommand]:
        """读取当前有效 MIT 指令，未收到或超时时返回零指令。"""
        with self._command_lock:
            if mit_command_expired(
                self._last_mit_command_time, self.mit_command_timeout_s
            ):
                return [ZERO_MIT_COMMAND] * len(self.motor_ids)
            return list(self._mit_commands)

    def _publish_joint_state(self) -> None:
        """发布 JointState，供可视化与上层控制使用。"""
        with self._state_lock:
            feedback = [self._feedback_by_id.get(motor_id) for motor_id in self.motor_ids]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [item.position if item else 0.0 for item in feedback]
        msg.velocity = [item.velocity if item else 0.0 for item in feedback]
        msg.effort = [item.torque if item else 0.0 for item in feedback]
        self.dm_state_publisher.publish(msg)
        if self.publish_robot_joint_states:
            self.robot_state_publisher.publish(msg)

    def _publish_status(self) -> None:
        """发布文本状态摘要并输出调试日志。"""
        now = time.monotonic()
        parts: List[str] = [
            f"enabled={int(self.enabled)}",
            f"mode={self._control_mode_label()}",
        ]
        with self._state_lock:
            feedback_by_id = dict(self._feedback_by_id)
            feedback_time_by_id = dict(self._feedback_time_by_id)
            last_commanded_torque = list(self._last_commanded_torque)

        cmd_torque = ",".join(f"{value:.2f}" for value in last_commanded_torque)
        parts.append(f"cmd_torque=[{cmd_torque}]")

        for motor_id in self.motor_ids:
            feedback = feedback_by_id.get(motor_id)
            feedback_time = feedback_time_by_id.get(motor_id)
            if feedback is None or feedback_time is None:
                parts.append(f"id{motor_id}=missing")
                continue
            age = now - feedback_time
            fresh = age <= self.feedback_timeout_s
            parts.append(
                f"id{motor_id}={feedback.error_name},fresh={int(fresh)},"
                f"age={age:.3f},mos={feedback.mos_temperature},"
                f"rotor={feedback.rotor_temperature}"
            )

        msg = String()
        msg.data = " ".join(parts)
        self.status_publisher.publish(msg)
        self.get_logger().debug(msg.data)

    def _control_mode_label(self) -> str:
        """返回当前控制阶段标签，便于状态话题调试。"""
        if not self.enabled:
            return "disabled"
        if self._has_fresh_mit_command():
            return "topic_mit"
        return "zero_mit"

    def _has_fresh_mit_command(self) -> bool:
        """检查当前缓存的上层 MIT 指令是否仍在有效期内。"""
        with self._command_lock:
            return not mit_command_expired(
                self._last_mit_command_time, self.mit_command_timeout_s
            )

    def _handle_usbcan_frame(self, frame: UsbCanFrame) -> None:
        """处理 USB-CAN 回传帧：优先寄存器应答，其次反馈帧。"""
        if (
            frame.command != 0x11
            or len(frame.payload) < 8
            or (not self.accept_any_master_id and frame.can_id != self.master_id)
        ):
            return

        with self._register_lock:
            has_pending_registers = len(self._pending_register_keys) > 0

        # 如果没有待处理的寄存器请求，此帧绝不可能是我们要等的回包，直接走正常解析
        if has_pending_registers:
            register_reply = parse_register_reply(frame.payload)
            if register_reply is not None:
                key = (
                    register_reply.motor_id,
                    register_reply.register_id,
                    register_reply.command,
                )
                with self._register_lock:
                    if key in self._pending_register_keys:
                        self._register_replies[key] = register_reply.value
                        self._pending_register_keys.discard(key)
                        # 匹配到寄存器 ACK 后直接返回，避免再按普通反馈解释该帧。
                        return

        try:
            feedback = parse_feedback(frame.payload, self.limits)
        except ValueError:
            return
        if feedback.motor_id not in self.motor_ids:
            return

        with self._state_lock:
            self._feedback_by_id[feedback.motor_id] = feedback
            self._feedback_time_by_id[feedback.motor_id] = time.monotonic()

    def _handle_mit_command(self, msg: Float32MultiArray) -> None:
        """订阅四电机 MIT 指令：[p, v, kp, kd, t_ff] * motor_count。"""
        values = [float(value) for value in msg.data]
        expected_length = len(self.motor_ids) * 5
        if len(values) != expected_length:
            self.get_logger().warning(
                f"Expected {expected_length} MIT command values "
                f"([p,v,kp,kd,t_ff] * {len(self.motor_ids)}), got {len(values)}."
            )
            return

        commands = [
            MITCommand(
                position=values[index],
                velocity=values[index + 1],
                kp=values[index + 2],
                kd=values[index + 3],
                torque=values[index + 4],
            )
            for index in range(0, expected_length, 5)
        ]
        with self._command_lock:
            self._mit_commands = commands
            self._last_mit_command_time = time.monotonic()

    def _handle_enable_all(self, _request, response):
        """服务回调：使能全部电机。"""
        self._send_special_to_all(make_enable_command())
        self.enabled = True
        response.success = True
        response.message = "Enable command sent to all motors."
        return response

    def _handle_disable_all(self, _request, response):
        """服务回调：失能全部电机。"""
        self._send_special_to_all(make_disable_command())
        self.enabled = False
        response.success = True
        response.message = "Disable command sent to all motors."
        return response

    def _handle_set_zero_all(self, _request, response):
        """服务回调：对全部电机执行当前位置清零。"""
        self._send_special_to_all(make_set_zero_command())
        response.success = True
        response.message = "Set-zero command sent to all motors."
        return response

    def _handle_clear_faults_all(self, _request, response):
        """服务回调：清除全部电机故障。"""
        self._send_special_to_all(make_clear_fault_command())
        response.success = True
        response.message = "Clear-fault command sent to all motors."
        return response


def main(args=None) -> None:
    """节点入口函数。"""
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
