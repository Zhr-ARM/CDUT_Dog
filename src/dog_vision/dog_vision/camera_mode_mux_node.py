#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


VALID_MODES = {"off", "yolo", "ocr"}
MODE_ALIASES = {
    "": "off",
    "none": "off",
    "idle": "off",
    "stop": "off",
    "false": "off",
    "0": "off",
    "true": "yolo",
    "1": "yolo",
}


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class CameraModeMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_mode_mux_node")
        self.declare_parameter("mode_topic", "/vision/mode")
        self.declare_parameter("legacy_enabled_topic", "/arm_vision_mode/enabled")
        self.declare_parameter("yolo_enabled_topic", "/vision/yolo_enabled")
        self.declare_parameter("ocr_enabled_topic", "/vision/ocr_enabled")
        self.declare_parameter("active_mode_topic", "/vision/active_mode")
        self.declare_parameter("initial_mode", "off")

        self.yolo_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("yolo_enabled_topic").value),
            _latched_qos(),
        )
        self.ocr_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("ocr_enabled_topic").value),
            _latched_qos(),
        )
        self.active_mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("active_mode_topic").value),
            _latched_qos(),
        )
        self.mode_sub = self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self.mode_callback,
            10,
        )
        self.legacy_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("legacy_enabled_topic").value),
            self.legacy_enabled_callback,
            10,
        )

        self.current_mode = ""
        self.apply_mode(str(self.get_parameter("initial_mode").value), source="initial")

    def mode_callback(self, msg: String) -> None:
        self.apply_mode(msg.data, source="mode")

    def legacy_enabled_callback(self, msg: Bool) -> None:
        self.apply_mode("yolo" if msg.data else "off", source="legacy")

    def normalize_mode(self, mode: str) -> str | None:
        normalized = mode.strip().lower()
        normalized = MODE_ALIASES.get(normalized, normalized)
        if normalized not in VALID_MODES:
            return None
        return normalized

    def apply_mode(self, requested_mode: str, source: str) -> None:
        mode = self.normalize_mode(requested_mode)
        if mode is None:
            self.get_logger().warning(f"Ignoring invalid vision mode from {source}: {requested_mode}")
            return
        if mode == self.current_mode:
            return
        self.current_mode = mode
        self.publish_bool(self.yolo_pub, mode == "yolo")
        self.publish_bool(self.ocr_pub, mode == "ocr")
        active_msg = String()
        active_msg.data = mode
        self.active_mode_pub.publish(active_msg)
        self.get_logger().info(f"vision mode -> {mode} ({source})")

    @staticmethod
    def publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraModeMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
