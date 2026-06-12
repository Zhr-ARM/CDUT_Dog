#!/usr/bin/env python3
"""
ROS2 语音播报节点 - 基于 pyttsx3 的中文 TTS 播报.

调用方式（向 /voice_broadcast 话题发布 std_msgs/String）:

  终端:
    ros2 topic pub /voice_broadcast std_msgs/msg/String "data: '你好世界'" -1

  Python:
    from std_msgs.msg import String
    msg = String(); msg.data = "你好世界"
    publisher.publish(msg)

  C++:
    auto msg = std_msgs::msg::String();
    msg.data = "你好世界";
    publisher->publish(msg);
"""

import queue
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceBroadcastNode(Node):
    """语音播报节点.

    订阅 /voice_broadcast 话题（std_msgs/String），接收到的文本以普通话语音播报。
    使用 pyttsx3 离线 TTS 引擎。
    播报完成后空闲超时自动退出，实现按需启停。
    """

    def __init__(self) -> None:
        super().__init__("voice_broadcast_node")

        # ---------- 参数 ----------
        self.declare_parameter("language", "zh")
        self.declare_parameter("rate", 180)          # 语速
        self.declare_parameter("volume", 1.0)        # 音量 0.0~1.0
        self.declare_parameter("queue_size", 50)     # 消息队列最大长度
        self.declare_parameter("idle_timeout", 5.0)  # 空闲超时自动退出(秒), 0=不退出

        self.language: str = self.get_parameter("language").value
        self.rate: int = self.get_parameter("rate").value
        self.volume: float = self.get_parameter("volume").value
        self._queue_max: int = self.get_parameter("queue_size").value
        self._idle_timeout: float = self.get_parameter("idle_timeout").value

        # ---------- pyttsx3 引擎初始化 ----------
        self._tts_engine: Optional[object] = None
        self._init_pyttsx3()

        # ---------- 消息队列 ----------
        self._msg_queue: queue.Queue = queue.Queue(maxsize=self._queue_max)
        self._speak_thread: Optional[threading.Thread] = None
        self._running = True
        self._idle_timer = None  # 空闲退出定时器

        # ---------- 订阅 ----------
        self._sub = self.create_subscription(
            String, "/voice_broadcast", self._msg_callback, 10
        )

        # ---------- 启动播放线程 ----------
        self._speak_thread = threading.Thread(target=self._speak_loop, daemon=True)
        self._speak_thread.start()

        self.get_logger().info(
            f"语音播报节点已启动 (语言: {self.language})"
        )
        # 启动时空闲超时也生效
        if self._idle_timeout > 0:
            self._schedule_idle_shutdown()

    # ------------------------------------------------------------------
    #  pyttsx3 引擎初始化
    # ------------------------------------------------------------------

    def _init_pyttsx3(self) -> None:
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.setProperty("rate", self.rate)
            engine.setProperty("volume", self.volume)
            # 设置中文普通话语音
            voices = engine.getProperty("voices")
            selected = None
            for v in voices:
                if "mandarin" in v.name.lower():
                    selected = v
                    break
            if selected is None:
                for v in voices:
                    if "chinese" in v.name.lower():
                        selected = v
                        break
            if selected is not None:
                engine.setProperty("voice", selected.id)
                self.get_logger().info(f"已选择语音: {selected.name}")
            else:
                self.get_logger().warn("未找到中文语音，使用系统默认")
            self._tts_engine = engine
            self.get_logger().info("TTS 引擎: pyttsx3")
        except Exception as e:
            self.get_logger().fatal(f"pyttsx3 初始化失败: {e}")
            raise RuntimeError("pyttsx3 不可用") from e

    # ------------------------------------------------------------------
    #  消息回调
    # ------------------------------------------------------------------

    def _msg_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self._cancel_idle_timer()
        try:
            self._msg_queue.put_nowait(text)
            self.get_logger().info(f"语音消息入队: {text[:30]}...")
        except queue.Full:
            self.get_logger().warning(
                f"消息队列已满({self._queue_max}), 丢弃: {text[:30]}..."
            )

    # ------------------------------------------------------------------
    #  播放线程
    # ------------------------------------------------------------------

    def _speak_loop(self) -> None:
        while self._running:
            try:
                text = self._msg_queue.get(timeout=0.5)
                self._speak(text)
                self._msg_queue.task_done()
                if self._msg_queue.empty() and self._idle_timeout > 0:
                    self._schedule_idle_shutdown()
            except queue.Empty:
                pass

    def _speak(self, text: str) -> None:
        """执行语音播报."""
        try:
            self._tts_engine.say(text)
            self._tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"pyttsx3 播报失败: {e}")

    # ------------------------------------------------------------------
    #  空闲自动退出
    # ------------------------------------------------------------------

    def _cancel_idle_timer(self) -> None:
        if self._idle_timer is not None:
            self.destroy_timer(self._idle_timer)
            self._idle_timer = None

    def _schedule_idle_shutdown(self) -> None:
        self._cancel_idle_timer()
        if self._idle_timeout > 0:
            self._idle_timer = self.create_timer(
                self._idle_timeout, self._on_idle_timeout
            )

    def _on_idle_timeout(self) -> None:
        self.get_logger().info(
            f"空闲 {self._idle_timeout:.0f}s，节点自动退出"
        )
        self._idle_timer = None
        self._running = False

    # ------------------------------------------------------------------
    #  生命周期
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._running = False
        self._cancel_idle_timer()
        if self._speak_thread and self._speak_thread.is_alive():
            self._speak_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceBroadcastNode()
    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    main()
