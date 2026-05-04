"""USB-CAN 串口通信封装。

职责:
- 发送上位机协议格式的 CAN 帧。
- 在后台线程持续接收并解析 USB-CAN 回传帧。
- 将解析结果以回调方式交给上层节点处理。
"""

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import serial
except ImportError as exc:
    # 延迟抛错：仅在 open() 时提示依赖缺失，便于单元测试或静态检查。
    serial = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


# 设备回传的单帧长度固定为 16 字节。
CAN_RX_FRAME_SIZE = 16


@dataclass(frozen=True)
class UsbCanFrame:
    """解析后的 USB-CAN 回传帧。"""

    command: int
    can_id: int
    payload: bytes
    is_extended: bool
    is_remote: bool


class UsbCanSerial:
    """USB-CAN 串口设备访问类。"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        frame_callback: Optional[Callable[[UsbCanFrame], None]] = None,
    ) -> None:
        """初始化串口参数与收发线程状态。"""
        self.port = port
        self.baudrate = baudrate
        self.frame_callback = frame_callback
        self._serial = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()

    def open(self) -> None:
        """打开串口并启动后台接收线程。"""
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Install it with: sudo apt install python3-serial"
            ) from _SERIAL_IMPORT_ERROR

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
        )
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        """停止接收线程并关闭串口。"""
        self._stop_event.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def set_can_bitrate(self, index: int) -> None:
        """发送设备定义的波特率配置命令。"""
        self._write(bytes([0x55, 0x05, index & 0xFF, 0xAA, 0x55]))

    def send_can_frame(
        self,
        can_id: int,
        payload: bytes,
        *,
        command: int = 0x01,
        send_times: int = 1,
        send_interval_100us: int = 0,
        is_extended: bool = False,
        is_remote: bool = False,
    ) -> None:
        """按 USB-CAN 设备协议封装并发送一帧 CAN 数据。"""
        if len(payload) > 8:
            raise ValueError("Classic CAN payload must be at most 8 bytes.")

        frame = bytearray(30)
        # 发送帧格式（30 字节）:
        # [0:2]   固定头 0x55 0xAA
        # [2]     帧长（固定 0x1E）
        # [3]     USB-CAN 命令字
        # [4:8]   发送次数（小端）
        # [8:12]  发送间隔（100us， 小端）
        # [12]    扩展帧标志
        # [13:17] CAN ID（小端）
        # [17]    远程帧标志
        # [18]    数据长度 DLC
        # [19:20]保留
        # [21:29]数据区（最多 8 字节）
        # [29]    CRC8
        frame[0] = 0x55
        frame[1] = 0xAA
        frame[2] = 0x1E
        frame[3] = command & 0xFF
        frame[4:8] = int(send_times).to_bytes(4, "little", signed=False)
        frame[8:12] = int(send_interval_100us).to_bytes(4, "little", signed=False)
        frame[12] = 0x01 if is_extended else 0x00
        frame[13:17] = int(can_id).to_bytes(4, "little", signed=False)
        frame[17] = 0x01 if is_remote else 0x00
        frame[18] = len(payload)
        frame[19] = 0x00
        frame[20] = 0x00
        frame[21 : 21 + len(payload)] = payload
        frame[29] = self.crc8(frame[:29])
        self._write(bytes(frame))

    @staticmethod
    def crc8(data: bytes) -> int:
        """计算设备协议使用的 CRC8 校验值。"""
        crc = 0
        for value in data:
            crc ^= value
            for _ in range(8):
                if crc & 0x01:
                    crc = ((crc >> 1) ^ 0x8C) & 0xFF
                else:
                    crc = (crc >> 1) & 0xFF
        return crc

    def _write(self, data: bytes) -> None:
        """线程安全写串口。"""
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("USB-CAN serial port is not open.")
        with self._write_lock:
            self._serial.write(data)

    def _rx_loop(self) -> None:
        """后台接收循环：读串口并持续喂给帧解析器。"""
        buffer = bytearray()
        while not self._stop_event.is_set():
            if not self._serial:
                time.sleep(0.01)
                continue

            chunk = self._serial.read(64)
            if not chunk:
                continue

            buffer.extend(chunk)
            self._consume_rx_buffer(buffer)

    def _consume_rx_buffer(self, buffer: bytearray) -> None:
        """从累积缓冲中切分并解析完整回传帧。"""
        while len(buffer) >= CAN_RX_FRAME_SIZE:
            header_index = buffer.find(b"\xAA")
            if header_index < 0:
                buffer.clear()
                return
            if header_index:
                del buffer[:header_index]
            if len(buffer) < CAN_RX_FRAME_SIZE:
                return
            if buffer[15] != 0x55:
                del buffer[0]
                continue

            raw = bytes(buffer[:CAN_RX_FRAME_SIZE])
            del buffer[:CAN_RX_FRAME_SIZE]
            frame = self._parse_rx_frame(raw)
            if frame and self.frame_callback:
                self.frame_callback(frame)

    @staticmethod
    def _parse_rx_frame(raw: bytes) -> Optional[UsbCanFrame]:
        """解析 16 字节回传帧，不合法时返回 None。"""
        if len(raw) != CAN_RX_FRAME_SIZE or raw[0] != 0xAA or raw[15] != 0x55:
            return None

        fmt = raw[2]
        data_len = fmt & 0x3F
        if data_len > 8:
            return None

        return UsbCanFrame(
            command=raw[1],
            can_id=int.from_bytes(raw[3:7], "little", signed=False),
            payload=bytes(raw[7 : 7 + data_len]),
            is_extended=bool(fmt & 0x40),
            is_remote=bool(fmt & 0x80),
        )
