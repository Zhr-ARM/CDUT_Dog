from __future__ import annotations

import platform
import shutil
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import numpy as np
from PIL import Image

from robocon_ocr.config import CameraConfig


def _build_fourcc(cv2_module, pixel_format: str) -> int:
    normalized = pixel_format.strip().upper()
    if len(normalized) != 4:
        raise ValueError("pixel_format must be a four-character code such as MJPG or YUYV")
    return cv2_module.VideoWriter_fourcc(*normalized)


def _first_video_device() -> str | None:
    for candidate in sorted(Path("/dev").glob("video*")):
        if candidate.exists():
            return str(candidate)
    return None


class USBCameraCapture:
    def __init__(self, config: CameraConfig) -> None:
        self.config = config

    def _import_cv2(self):
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "未安装 OpenCV。请先执行 `pip install -r requirements.txt`。"
            ) from exc
        return cv2

    def _open_capture(self):
        cv2 = self._import_cv2()
        source = self._capture_source()
        try:
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
        except TypeError:
            capture = cv2.VideoCapture(source)
        if not capture.isOpened() and self.config.device:
            capture.release()
            capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            raise RuntimeError(f"无法打开 USB 摄像头设备 {self._camera_device_path()}")

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.height))
        capture.set(cv2.CAP_PROP_FPS, float(self.config.fps))
        capture.set(cv2.CAP_PROP_FOURCC, float(_build_fourcc(cv2, self.config.pixel_format)))
        self._apply_camera_controls()
        return cv2, capture

    def _capture_source(self) -> str | int:
        if self.config.device:
            if self.config.device.startswith("/dev/") and not Path(self.config.device).exists():
                fallback = _first_video_device()
                if fallback is not None:
                    self._warn(f"摄像头设备 {self.config.device} 不存在，改用 {fallback}。")
                    return fallback
            return self.config.device
        return self.config.device_index

    def _camera_device_path(self) -> str:
        if self.config.device:
            if self.config.device.startswith("/dev/") and not Path(self.config.device).exists():
                fallback = _first_video_device()
                if fallback is not None:
                    return fallback
            return self.config.device
        return f"/dev/video{self.config.device_index}"

    def _warn(self, message: str) -> None:
        print(f"[camera-init] {message}", file=sys.stderr)

    def _camera_controls_in_order(self) -> list[tuple[str, int]]:
        controls: list[tuple[str, int]] = []
        for name in (
            "white_balance_automatic",
            "white_balance_temperature",
            "focus_automatic_continuous",
            "focus_absolute",
            "auto_exposure",
            "exposure_dynamic_framerate",
            "exposure_time_absolute",
            "gain",
            "brightness",
            "contrast",
            "saturation",
            "sharpness",
            "gamma",
            "backlight_compensation",
            "power_line_frequency",
        ):
            value = getattr(self.config, name)
            if value is None:
                continue
            controls.append((name, int(value)))
        return controls

    def _run_v4l2_set_ctrl(self, control_name: str, value: int) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                self._camera_device_path(),
                f"--set-ctrl={control_name}={value}",
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    def _apply_camera_controls(self) -> None:
        if platform.system() != "Linux":
            self._warn("当前平台不是 Linux，跳过 v4l2 控制项初始化。")
            return
        if self.config.device and not self.config.device.startswith("/dev/"):
            return

        if shutil.which("v4l2-ctl") is None:
            self._warn("系统中未找到 v4l2-ctl，跳过摄像头固定参数初始化。")
            return

        for control_name, value in self._camera_controls_in_order():
            result = self._run_v4l2_set_ctrl(control_name, value)
            if result.returncode == 0:
                continue

            output = (result.stderr or result.stdout).strip()
            if "unknown control" in output.lower():
                self._warn(f"设备不支持控制项 `{control_name}`，已跳过。")
            elif "invalid argument" in output.lower():
                self._warn(f"控制项 `{control_name}` 的值 `{value}` 不被当前设备接受：{output}")
            else:
                self._warn(f"设置控制项 `{control_name}={value}` 失败：{output or 'unknown error'}")

    def open(self):
        return self._open_capture()

    def capture_frame(self) -> Image.Image:
        cv2, capture = self._open_capture()
        try:
            return self._read_stable_frame(cv2, capture, discard_frames=max(1, self.config.warmup_frames))
        finally:
            capture.release()

    def stream_frames(self) -> Iterator[tuple[int, Image.Image]]:
        cv2, capture = self._open_capture()
        try:
            frame_index = 0
            warmup = max(1, self.config.warmup_frames)
            yield frame_index, self._read_stable_frame(cv2, capture, discard_frames=warmup)
            frame_index += 1

            while self.config.max_frames is None or frame_index < self.config.max_frames:
                if self.config.interval_ms > 0:
                    time.sleep(self.config.interval_ms / 1000.0)
                yield frame_index, self._read_stable_frame(cv2, capture, discard_frames=1)
                frame_index += 1
        finally:
            capture.release()

    def stream_raw_frames(self) -> Iterator[tuple[int, np.ndarray]]:
        cv2, capture = self._open_capture()
        try:
            frame_index = 0
            warmup = max(1, self.config.warmup_frames)
            self._read_raw_frame(capture, discard_frames=warmup)

            while self.config.max_frames is None or frame_index < self.config.max_frames:
                if self.config.interval_ms > 0 and not self.config.async_latest_frame:
                    time.sleep(self.config.interval_ms / 1000.0)
                raw_frame = self._read_raw_frame(capture, discard_frames=1)
                yield frame_index, raw_frame
                frame_index += 1
        finally:
            capture.release()

    def _read_stable_frame(self, cv2_module, capture, discard_frames: int) -> Image.Image:
        frame = self._read_raw_frame(capture, discard_frames)
        rgb_frame = cv2_module.cvtColor(frame, cv2_module.COLOR_BGR2RGB)
        image = Image.fromarray(rgb_frame)
        if self.config.save_frame is not None:
            save_path = Path(self.config.save_frame).expanduser()
            save_path.parent.mkdir(parents=True, exist_ok=True)
            image.save(save_path)
        return image

    def _read_raw_frame(self, capture, discard_frames: int) -> np.ndarray:
        deadline = time.monotonic() + (self.config.capture_timeout_ms / 1000.0)
        frame = None
        frames_to_read = max(1, discard_frames)

        while time.monotonic() < deadline:
            ok, raw_frame = capture.read()
            if not ok:
                continue
            frame = raw_frame
            frames_to_read -= 1
            if frames_to_read <= 0:
                break

        if frame is None or frames_to_read > 0:
            raise RuntimeError("在超时时间内未能从 USB 摄像头读取到稳定画面")
        return frame
