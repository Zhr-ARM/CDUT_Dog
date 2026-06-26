#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from dog_bringup.msg import OcrResult
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String


BACKENDS = {"api", "onnx"}
FALLBACK_BACKEND = "onnx"
RESTART_PARAMETERS = {
    "backend",
    "device",
    "device_index",
    "width",
    "height",
    "fps",
    "pixel_format",
    "show_image",
    "warmup_frames",
    "capture_timeout_ms",
    "max_frames",
    "filter_consensus",
    "onnx_python",
    "api_python",
    "onnx_model_path",
    "onnx_dict_path",
    "api_provider",
    "api_model",
    "api_key",
    "api_base_url",
    "api_timeout",
    "api_max_tokens",
    "env_file",
}

IGNORED_WORKER_STDERR_PATTERNS = (
    "QFontDatabase: Cannot find font directory",
    "Note that Qt no longer ships fonts",
    "GPU device discovery failed",
    "device_discovery.cc",
)


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def _find_workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "dog_vision").is_dir():
            return parent
    return Path.cwd()


def _default_python(name: str) -> str:
    return str(_find_workspace_root() / name / "bin" / "python")


def _default_env_file() -> str:
    path = _find_workspace_root() / ".env"
    return str(path) if path.is_file() else ""


class OcrRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__("ocr_recognition_node")
        package_share = Path(get_package_share_directory("dog_vision"))

        self.declare_parameter("backend", "api")
        self.declare_parameter("enabled_topic", "/vision/ocr_enabled")
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("result_topic", "/vision/ocr_result")
        self.declare_parameter("mod4_topic", "/vision/ocr_mod4")
        self.declare_parameter("status_topic", "/vision/ocr_status")
        self.declare_parameter("backend_topic", "/vision/ocr_backend")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("device_index", 0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("show_image", False)
        self.declare_parameter("warmup_frames", 5)
        self.declare_parameter("capture_timeout_ms", 3000)
        self.declare_parameter("max_frames", 0)
        self.declare_parameter("filter_consensus", 3)
        self.declare_parameter("onnx_python", _default_python(".venv-ocr-onnx"))
        self.declare_parameter("api_python", _default_python(".venv-ocr-onnx"))
        self.declare_parameter(
            "onnx_model_path",
            str(package_share / "models" / "ocr" / "PP-OCRv5_server_rec.onnx"),
        )
        self.declare_parameter("onnx_dict_path", str(package_share / "models" / "ocr" / "dict.txt"))
        self.declare_parameter("api_provider", "moonshot")
        self.declare_parameter("api_model", "")
        self.declare_parameter("api_key", "")
        self.declare_parameter("api_base_url", "")
        self.declare_parameter("api_timeout", 5.0)
        self.declare_parameter("api_max_tokens", 128)
        self.declare_parameter("env_file", _default_env_file())
        self.declare_parameter("allow_api_fallback", False)
        self.declare_parameter("worker_stop_timeout", 5.0)

        self.result_pub = self.create_publisher(
            OcrResult,
            str(self.get_parameter("result_topic").value),
            _latched_qos(),
        )
        self.mod4_pub = self.create_publisher(Int32, str(self.get_parameter("mod4_topic").value), 10)
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            _latched_qos(),
        )
        self.backend_pub = self.create_publisher(
            String,
            str(self.get_parameter("backend_topic").value),
            _latched_qos(),
        )
        self.enabled_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("enabled_topic").value),
            self.enabled_callback,
            10,
        )

        self._lock = threading.RLock()
        self._process: subprocess.Popen[str] | None = None
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._enabled = bool(self.get_parameter("start_enabled").value)
        self._restart_requested = False
        self._active_backend: str | None = None
        self._fallback_attempted = False

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.watch_timer = self.create_timer(0.5, self.watch_worker)
        self.publish_status("idle")
        self.publish_backend("none")

        if self._enabled:
            self.start_worker()

    def parameters_callback(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "backend" and str(parameter.value).strip().lower() not in BACKENDS:
                return SetParametersResult(
                    successful=False,
                    reason=f"backend must be one of {sorted(BACKENDS)}",
                )
            if parameter.name in RESTART_PARAMETERS:
                self._restart_requested = True
        return SetParametersResult(successful=True)

    def enabled_callback(self, msg: Bool) -> None:
        with self._lock:
            if bool(msg.data) == self._enabled:
                return
            self._enabled = bool(msg.data)
            if self._enabled:
                self._fallback_attempted = False
                self.start_worker()
            else:
                self.stop_worker()

    def watch_worker(self) -> None:
        with self._lock:
            if self._restart_requested and self._enabled:
                self._restart_requested = False
                self._fallback_attempted = False
                self.stop_worker()
                self.start_worker()
                return
            process = self._process
            if process is None:
                return
            return_code = process.poll()
            if return_code is None:
                return
            self._process = None
            failed_backend = self._active_backend or str(self.get_parameter("backend").value)
            self._active_backend = None
            if self._enabled:
                if return_code != 0:
                    if self.try_start_fallback(f"worker exited with code {return_code}", failed_backend):
                        return
                    if not self._enabled:
                        return
                self._enabled = False
                if return_code == 0:
                    self.publish_status("stopped")
                    self.publish_backend("none")
                else:
                    self.publish_status(f"error:{failed_backend}: worker exited with code {return_code}")
            else:
                self.publish_status("stopped")
                self.publish_backend("none")

    def start_worker(self) -> None:
        if self._process is not None and self._process.poll() is None:
            return

        backend = str(self.get_parameter("backend").value).strip().lower()
        self._start_worker_for_backend(backend)

    def _start_worker_for_backend(self, backend: str) -> bool:
        if backend not in BACKENDS:
            self.publish_status(f"error: backend must be one of {sorted(BACKENDS)}")
            self._enabled = False
            return False

        python_path = self.python_for_backend(backend)
        if not Path(python_path).is_file():
            if self.try_start_fallback(f"python not found: {python_path}", backend):
                return True
            self.publish_status(f"error: python not found: {python_path}")
            self._enabled = False
            return False

        cmd = self.build_worker_command(python_path, backend)
        env = self.build_worker_env()
        self.publish_backend(backend)
        self.publish_status(f"starting:{backend}")
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
        except OSError as exc:
            if self.try_start_fallback(f"failed to start worker: {exc}", backend):
                return True
            self.publish_status(f"error: failed to start worker: {exc}")
            self._process = None
            self._enabled = False
            return False

        self._active_backend = backend

        self._stdout_thread = threading.Thread(
            target=self.read_stdout,
            args=(self._process,),
            name="ocr-worker-stdout",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self.read_stderr,
            args=(self._process,),
            name="ocr-worker-stderr",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()
        return True

    def try_start_fallback(self, reason: str, failed_backend: str) -> bool:
        allow_fallback = bool(self.get_parameter("allow_api_fallback").value)
        if not allow_fallback or failed_backend != "api" or self._fallback_attempted:
            return False
        self._fallback_attempted = True
        self.get_logger().warning(
            f"OCR api backend unavailable, falling back to {FALLBACK_BACKEND}: {reason}"
        )
        self.publish_status(f"fallback: api -> {FALLBACK_BACKEND}: {reason}")
        return self._start_worker_for_backend(FALLBACK_BACKEND)

    def stop_worker(self) -> None:
        process = self._process
        if process is None:
            self._active_backend = None
            self._fallback_attempted = False
            self.publish_status("stopped")
            self.publish_backend("none")
            return
        if process.poll() is not None:
            self._process = None
            self._active_backend = None
            self.publish_status("stopped")
            self.publish_backend("none")
            return

        self.publish_status("stopping")
        timeout = float(self.get_parameter("worker_stop_timeout").value)
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        finally:
            self._process = None
            self._active_backend = None
            self._fallback_attempted = False
            self.publish_status("stopped")
            self.publish_backend("none")

    def python_for_backend(self, backend: str) -> str:
        if backend == "api":
            return str(self.get_parameter("api_python").value)
        return str(self.get_parameter("onnx_python").value)

    def build_worker_command(self, python_path: str, backend: str) -> list[str]:
        cmd = [
            python_path,
            "-m",
            "dog_vision.ocr_worker_main",
            "--backend",
            backend,
            "--device",
            str(self.get_parameter("device").value),
            "--device-index",
            str(int(self.get_parameter("device_index").value)),
            "--width",
            str(int(self.get_parameter("width").value)),
            "--height",
            str(int(self.get_parameter("height").value)),
            "--fps",
            str(float(self.get_parameter("fps").value)),
            "--pixel-format",
            str(self.get_parameter("pixel_format").value),
            "--show-image",
            str(bool(self.get_parameter("show_image").value)).lower(),
            "--warmup-frames",
            str(int(self.get_parameter("warmup_frames").value)),
            "--capture-timeout-ms",
            str(int(self.get_parameter("capture_timeout_ms").value)),
            "--max-frames",
            str(int(self.get_parameter("max_frames").value)),
            "--filter-consensus",
            str(int(self.get_parameter("filter_consensus").value)),
            "--onnx-model-path",
            str(self.get_parameter("onnx_model_path").value),
            "--onnx-dict-path",
            str(self.get_parameter("onnx_dict_path").value),
            "--api-provider",
            str(self.get_parameter("api_provider").value),
            "--api-timeout",
            str(float(self.get_parameter("api_timeout").value)),
            "--api-max-tokens",
            str(int(self.get_parameter("api_max_tokens").value)),
        ]
        for parameter_name, flag in (
            ("api_model", "--api-model"),
            ("api_key", "--api-key"),
            ("api_base_url", "--api-base-url"),
            ("env_file", "--env-file"),
        ):
            value = str(self.get_parameter(parameter_name).value)
            if value:
                cmd.extend([flag, value])
        return cmd

    def build_worker_env(self) -> dict[str, str]:
        env = os.environ.copy()
        module_parent = str(Path(__file__).resolve().parents[1])
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = module_parent + (os.pathsep + existing if existing else "")
        env["PYTHONUNBUFFERED"] = "1"
        font_candidates = (
            Path("/usr/share/fonts/truetype/dejavu"),
            Path("/usr/share/fonts/truetype/liberation2"),
            Path("/usr/share/fonts/truetype"),
        )
        for font_dir in font_candidates:
            if font_dir.is_dir():
                env.setdefault("QT_QPA_FONTDIR", str(font_dir))
                break
        env.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts=false")
        env.setdefault("ORT_LOG_SEVERITY_LEVEL", "3")
        return env

    def read_stdout(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            text = line.strip()
            if not text:
                continue
            if not rclpy.ok():
                return
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                self.get_logger().warning(f"bad OCR worker stdout: {text[:200]}")
                continue
            self.handle_worker_payload(payload)

    def read_stderr(self, process: subprocess.Popen[str]) -> None:
        if process.stderr is None:
            return
        for line in process.stderr:
            text = line.strip()
            if not text:
                continue
            if self.should_ignore_worker_stderr(text):
                continue
            if not rclpy.ok():
                return
            self.get_logger().warning(f"OCR worker: {text}")

    def should_ignore_worker_stderr(self, text: str) -> bool:
        return any(pattern in text for pattern in IGNORED_WORKER_STDERR_PATTERNS)

    def handle_worker_payload(self, payload: dict[str, Any]) -> None:
        payload_type = payload.get("type")
        if payload_type == "status":
            status = str(payload.get("status", "running"))
            backend = str(payload.get("backend") or self._active_backend or "")
            if status == "running" and backend:
                self.publish_backend(backend)
                self.publish_status(f"running:{backend}")
            else:
                self.publish_status(status)
            return
        if payload_type != "result":
            self.get_logger().warning(f"unknown OCR worker payload type: {payload_type}")
            return
        self.publish_result(payload)

    def publish_result(self, payload: dict[str, Any]) -> None:
        if not rclpy.ok():
            return
        msg = OcrResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("device").value)
        msg.expression = str(payload.get("expression") or "")
        answer = payload.get("answer")
        answer_mod_4 = payload.get("answer_mod_4")
        msg.has_answer = answer is not None
        msg.answer = int(answer) if answer is not None else 0
        msg.answer_mod_4 = int(answer_mod_4) if answer_mod_4 is not None else 0
        msg.is_valid = bool(payload.get("is_valid", False))
        msg.confidence = float(payload.get("confidence") or 0.0)
        msg.backend = str(payload.get("backend") or "")
        msg.error = str(payload.get("error") or "")
        try:
            self.result_pub.publish(msg)
        except Exception:
            return

        if msg.is_valid and msg.has_answer:
            mod_msg = Int32()
            mod_msg.data = int(msg.answer_mod_4)
            try:
                self.mod4_pub.publish(mod_msg)
            except Exception:
                return

    def publish_backend(self, backend: str) -> None:
        if not rclpy.ok():
            return
        msg = String()
        msg.data = backend
        try:
            self.backend_pub.publish(msg)
        except Exception:
            return

    def publish_status(self, status: str) -> None:
        if not rclpy.ok():
            return
        msg = String()
        msg.data = status
        try:
            self.status_pub.publish(msg)
            self.get_logger().info(f"OCR status: {status}")
        except Exception:
            return

    def destroy_node(self) -> None:
        with self._lock:
            self._enabled = False
            self.stop_worker()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OcrRecognitionNode()
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
