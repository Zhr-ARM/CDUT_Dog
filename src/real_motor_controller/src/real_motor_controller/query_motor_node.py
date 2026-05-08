"""电机在线扫描节点。

该节点通过发送轻量探测命令并等待反馈，判断指定 motor_id 是否在线。
"""

import threading
import time
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from real_motor_controller.dm_protocol import MotorFeedback, make_disable_command, parse_feedback
from real_motor_controller.ros_params import declare_parameters, load_parameters
from real_motor_controller.usbcan_serial import UsbCanFrame, UsbCanSerial


# 扫描节点参数默认值。
QUERY_PARAMS = {
    "serial_port": "/dev/ttyACM0",
    "serial_baudrate": 921600,
    "can_bitrate_index": 0,
    "set_can_bitrate_on_start": True,
    "motor_ids": [1, 2, 3, 4],
    "master_id": 0,
    "accept_any_master_id": True,
    "log_rx_frames": False,
    "probe_attempts": 3,
    "probe_timeout_s": 0.2,
    "probe_gap_s": 0.03,
    "usbcan_send_cmd": 0x01,
}

# 发送探测帧时统一使用单次发送配置。
SEND_ONCE = {
    "send_times": 1,
    "send_interval_100us": 0,
    "is_extended": False,
    "is_remote": False,
}


@dataclass(frozen=True)
class ObservedFeedback:
    """缓存一次观测到的反馈及其来源 CAN ID。"""

    feedback: MotorFeedback
    feedback_can_id: int


class QueryMotorNode(Node):
    """查询电机是否在线并发布扫描结果的 ROS2 节点。"""

    serial_port: str
    serial_baudrate: int
    can_bitrate_index: int
    set_can_bitrate_on_start: bool
    motor_ids: list
    master_id: int
    accept_any_master_id: bool
    log_rx_frames: bool
    probe_attempts: int
    probe_timeout_s: float
    probe_gap_s: float
    usbcan_send_cmd: int

    def __init__(self) -> None:
        """初始化节点并执行首轮电机在线扫描。

        初始化流程:
        1) 声明并加载参数。
        2) 建立线程安全的反馈缓存。
        3) 创建扫描结果发布器。
        4) 打开 USB-CAN 串口并按需设置 CAN 波特率。
        5) 立即执行一次扫描并发布结果。
        """
        super().__init__("real_query_motor_node")

        # 先声明参数，再读取参数值，避免未声明参数访问异常。
        declare_parameters(self, QUERY_PARAMS)
        self._load_parameters()

        # 反馈缓存由接收线程和扫描逻辑共享，需加锁保证并发安全。
        self._feedback_lock = threading.Lock()
        self._feedback_by_id: Dict[int, ObservedFeedback] = {}

        # 创建对外接口：扫描结果话题。
        self.result_publisher = self.create_publisher(String, "query_result", 10)

        # 初始化并打开 USB-CAN 通道，回传帧通过回调进入解析流程。
        self.bus = UsbCanSerial(
            self.serial_port,
            self.serial_baudrate,
            frame_callback=self._handle_usbcan_frame,
        )
        self.bus.open()
        self.get_logger().info(
            f"Opened USB-CAN serial port {self.serial_port} at {self.serial_baudrate}."
        )

        # 部分设备上电后需要显式发送波特率命令，确保总线参数一致。
        if self.set_can_bitrate_on_start:
            self.bus.set_can_bitrate(self.can_bitrate_index)
            self.get_logger().info(
                f"Sent USB-CAN bitrate index {self.can_bitrate_index} command."
            )
            time.sleep(0.2)

        # 节点启动即执行一次扫描，便于上层快速获得在线状态。
        results = self.scan_motors()
        self._publish_result(results)

    def _load_parameters(self) -> None:
        """加载并缓存参数。"""
        self.__dict__.update(load_parameters(self, QUERY_PARAMS))

    def scan_motors(self) -> Dict[int, bool]:
        """按 motor_id 顺序执行探测扫描。

        返回:
        - 字典 {motor_id: 是否探测到反馈}
        """
        results: Dict[int, bool] = {}
        # 采用失能命令作为探测负载：兼容性高且不会触发复杂控制行为。
        probe_payload = make_disable_command()

        with self._feedback_lock:
            self._feedback_by_id.clear()

        for motor_id in self.motor_ids:
            found = False
            for attempt in range(1, self.probe_attempts + 1):
                self._send_probe(motor_id, probe_payload)

                deadline = time.monotonic() + self.probe_timeout_s
                while time.monotonic() < deadline:
                    if self._has_feedback(motor_id):
                        found = True
                        break
                    time.sleep(0.005)

                if found:
                    break

                self.get_logger().debug(
                    f"No feedback from motor {motor_id} on attempt {attempt}."
                )
                time.sleep(self.probe_gap_s)

            results[motor_id] = found
            if found:
                observed = self._get_feedback(motor_id)
                feedback = observed.feedback
                self.get_logger().info(
                    f"Motor ID {motor_id}: FOUND, state={feedback.error_name}, "
                    f"master_id=0x{observed.feedback_can_id:X}, "
                    f"mos={feedback.mos_temperature}C, rotor={feedback.rotor_temperature}C"
                )
            else:
                self.get_logger().warning(f"Motor ID {motor_id}: NOT FOUND")

        return results

    def _send_probe(self, motor_id: int, payload: bytes) -> None:
        """向指定电机发送一次探测帧。"""
        self.bus.send_can_frame(
            motor_id, payload, command=self.usbcan_send_cmd, **SEND_ONCE
        )

    def destroy_node(self) -> None:
        """关闭通信资源并销毁节点。"""
        if hasattr(self, "bus"):
            self.bus.close()
        super().destroy_node()

    def _handle_usbcan_frame(self, frame: UsbCanFrame) -> None:
        """处理回传帧，仅保留符合过滤条件的反馈数据。"""
        if self.log_rx_frames:
            self.get_logger().info(
                f"RX usbcan_cmd=0x{frame.command:02X}, can_id=0x{frame.can_id:X}, "
                f"data={frame.payload.hex(' ')}"
            )

        if (
            frame.command != 0x11
            or len(frame.payload) < 8
            or (not self.accept_any_master_id and frame.can_id != self.master_id)
        ):
            return

        try:
            feedback = parse_feedback(frame.payload)
        except ValueError:
            return

        with self._feedback_lock:
            self._feedback_by_id[feedback.motor_id] = ObservedFeedback(
                feedback=feedback,
                feedback_can_id=frame.can_id,
            )

    def _has_feedback(self, motor_id: int) -> bool:
        """检查指定电机是否已有反馈缓存。"""
        with self._feedback_lock:
            return motor_id in self._feedback_by_id

    def _get_feedback(self, motor_id: int) -> ObservedFeedback:
        """读取指定电机的反馈缓存。"""
        with self._feedback_lock:
            return self._feedback_by_id[motor_id]

    def _publish_result(self, results: Dict[int, bool]) -> None:
        """发布扫描结果字符串。"""
        msg = String()
        msg.data = self._format_result(results)
        self.result_publisher.publish(msg)

    def _format_result(self, results: Dict[int, bool]) -> str:
        """将扫描结果格式化为紧凑文本。"""
        return " ".join(
            f"id{motor_id}={'found' if results.get(motor_id, False) else 'missing'}"
            for motor_id in self.motor_ids
        )


def main(args=None) -> None:
    """节点入口函数。"""
    rclpy.init(args=args)
    node = None
    try:
        node = QueryMotorNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
