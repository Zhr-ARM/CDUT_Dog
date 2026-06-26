from __future__ import annotations

import threading
import time
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from robocon_ocr.camera_tuning import DEFAULT_CAMERA_TUNING
from robocon_ocr.config import CameraConfig, OCRConfig, PipelineConfig
from robocon_ocr.image_recognition.factory import create_recognizer
from robocon_ocr.recognition_filter import RecognitionFilter
from robocon_ocr.recognition_output import RecognitionOutput
from robocon_ocr.staged_pipeline import context_to_record, run_camera_pipeline_frame
from robocon_ocr.vision_capture.usb_camera import USBCameraCapture


DEBUG_FONT_CANDIDATES = (
    Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    Path("/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf"),
    Path("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"),
    Path("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"),
)


# ---------------------------------------------------------------------------
# Shared frame buffer (also used by cli.py)
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class LatestFrameBuffer:
    """Thread-safe single-slot buffer for the most recent camera frame."""

    frame_index: int = -1
    frame_bgr: np.ndarray | None = None
    captured_at: float = 0.0
    stopped: bool = False
    error: Exception | None = None
    condition: threading.Condition = field(default_factory=threading.Condition)

    def publish(self, frame_index: int, frame_bgr: np.ndarray) -> None:
        with self.condition:
            self.frame_index = frame_index
            self.frame_bgr = frame_bgr.copy()
            self.captured_at = time.monotonic()
            self.condition.notify_all()

    def stop(self, error: Exception | None = None) -> None:
        with self.condition:
            self.stopped = True
            self.error = error
            self.condition.notify_all()

    def wait_for_next(self, last_processed_index: int) -> tuple[int, np.ndarray, float] | None:
        with self.condition:
            while True:
                if self.error is not None:
                    raise RuntimeError("摄像头采集线程异常退出") from self.error
                if self.frame_bgr is not None and self.frame_index > last_processed_index:
                    return self.frame_index, self.frame_bgr.copy(), self.captured_at
                if self.stopped:
                    return None
                self.condition.wait(timeout=0.1)


# ---------------------------------------------------------------------------
# Camera session
# ---------------------------------------------------------------------------


class CameraRecognitionSession:
    """High-level session for real-time camera OCR recognition.

    Parent projects use this as the single entry point. Two backends are
    supported: ``"api"`` (default) and ``"onnx"``.

    **Callback style** (recommended)::

        def on_result(output: RecognitionOutput) -> None:
            print(f"answer={output.answer}  mod4={output.answer_mod_4}")

        session = CameraRecognitionSession(backend="api", on_result=on_result)
        session.start()  # blocks until stop() or KeyboardInterrupt

    **Semaphore / polling style**::

        session = CameraRecognitionSession(backend="api")
        session.start_async()
        while True:
            if session.result_ready.wait(timeout=1.0):
                output = session.latest_output
                print(f"answer={output.answer}  mod4={output.answer_mod_4}")
                session.result_ready.clear()
    """

    def __init__(
        self,
        backend: str = "api",
        on_result: Callable[[RecognitionOutput], None] | None = None,
        filter_consensus: int = 3,
        camera_config: CameraConfig | None = None,
        ocr_config: OCRConfig | None = None,
        show_image: bool = False,
        **camera_overrides,
    ) -> None:
        """
        Args:
            backend: One of ``"api"`` or ``"onnx"``.
            on_result: Optional callback invoked for each confirmed result.
            filter_consensus: Consecutive-match threshold for local models
                (ignored for ``api`` backend).
            camera_config: Optional full ``CameraConfig``; defaults to
                ``DEFAULT_CAMERA_TUNING`` merged with any ``camera_overrides``.
            ocr_config: Optional full ``OCRConfig``; useful for parent projects
                that need absolute model paths or API provider settings.
            show_image: Show a live OpenCV debug window with ROI and OCR text.
            **camera_overrides: Individual camera-config overrides, e.g.
                ``device_index=2``, ``width=1280``, ``height=720``.
        """
        if backend not in {"api", "onnx"}:
            raise ValueError(f"Unsupported backend: {backend}")

        self._backend = backend
        if ocr_config is None:
            ocr_config = OCRConfig(backend=backend, warmup=True)
        else:
            ocr_config.backend = backend
            ocr_config.warmup = True
        self._pipeline_config = PipelineConfig(dataset_dir=Path("."), ocr=ocr_config)

        # Camera config: defaults + overrides
        if camera_config is None:
            camera_config = CameraConfig()
        for attr_name in dir(DEFAULT_CAMERA_TUNING):
            if attr_name.startswith("_"):
                continue
            default_val = getattr(DEFAULT_CAMERA_TUNING, attr_name)
            if getattr(camera_config, attr_name) == getattr(CameraConfig(), attr_name):
                setattr(camera_config, attr_name, default_val)
        for key, value in camera_overrides.items():
            if hasattr(camera_config, key):
                setattr(camera_config, key, value)
        self._camera_config = camera_config
        self._show_image = show_image
        self._display_disabled = False
        self._debug_font = self._load_debug_font()

        self._on_result = on_result
        self._filter = RecognitionFilter(
            backend=backend,
            consensus=filter_consensus,
            on_emit=self._on_emit,
        )

        # Public result access
        self.result_ready = threading.Event()
        self.latest_output: RecognitionOutput | None = None
        self._output_lock = threading.Lock()

        # Internal state
        self._recognizer = create_recognizer(ocr_config)
        self._running = False
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._error: Exception | None = None

    @property
    def error(self) -> Exception | None:
        return self._error

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the camera session (blocking). Runs until ``stop()`` is
        called from another thread or ``KeyboardInterrupt``."""
        if self._running:
            return
        self._run()

    def start_async(self) -> None:
        """Start the camera session in a background daemon thread."""
        if self._running:
            return
        self._thread = threading.Thread(target=self._run, name="camera-session", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the session to stop and wait for the background thread."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _load_debug_font(self) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
        for font_path in DEBUG_FONT_CANDIDATES:
            if font_path.is_file():
                try:
                    return ImageFont.truetype(str(font_path), size=26)
                except OSError:
                    continue
        return ImageFont.load_default()

    def _on_emit(self, output: RecognitionOutput) -> None:
        with self._output_lock:
            self.latest_output = output
            self.result_ready.set()
        if self._on_result:
            self._on_result(output)

    def _run(self) -> None:
        self._running = True
        self._stop_event.clear()
        self._error = None
        camera = USBCameraCapture(self._camera_config)
        self._recognizer.warmup()
        buffer = LatestFrameBuffer()

        last_processed_index = -1

        def capture_worker() -> None:
            try:
                for frame_index, frame_bgr in camera.stream_raw_frames():
                    if self._stop_event.is_set():
                        break
                    buffer.publish(frame_index, frame_bgr)
            except Exception as exc:
                self._error = exc
                buffer.stop(exc)
            else:
                buffer.stop()

        def ocr_worker() -> None:
            nonlocal last_processed_index
            try:
                while not self._stop_event.is_set():
                    item = buffer.wait_for_next(last_processed_index)
                    if item is None:
                        break
                    frame_index, frame_bgr, _captured_at = item
                    image_name = f"camera_{self._camera_config.device_index}_{frame_index:06d}.png"
                    image_rgb = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
                    context = run_camera_pipeline_frame(
                        frame=image_rgb,
                        image_name=image_name,
                        config=self._pipeline_config,
                        recognizer=self._recognizer,
                    )
                    last_processed_index = frame_index
                    record = context_to_record(context)
                    if not self._show_debug_frame(frame_bgr, record):
                        self._stop_event.set()
                        buffer.stop()
                        break
                    self._process_record(record)
            except Exception as exc:
                self._error = exc
                self._stop_event.set()
                buffer.stop(exc)
            finally:
                self._close_debug_window()

        capture_thread = threading.Thread(target=capture_worker, name="cam-cap", daemon=True)
        ocr_thread = threading.Thread(target=ocr_worker, name="cam-ocr", daemon=True)
        capture_thread.start()
        ocr_thread.start()

        try:
            capture_thread.join()
            ocr_thread.join()
        except KeyboardInterrupt:
            self._stop_event.set()
            buffer.stop()
            capture_thread.join()
            ocr_thread.join()
        finally:
            self._running = False

    def _show_debug_frame(self, frame_bgr: np.ndarray, record) -> bool:
        if not self._show_image or self._display_disabled:
            return True

        try:
            display = frame_bgr.copy()
            if record.roi_quad is not None:
                points = np.array(record.roi_quad, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(display, [points], isClosed=True, color=(0, 255, 0), thickness=2)

            parsed = record.parsed
            lines = [
                f"backend: {record.ocr.backend}",
                f"raw: {record.ocr.raw_text or '-'}",
                f"expr: {parsed.expression or '-'}",
                f"answer: {parsed.answer if parsed.answer is not None else '-'}",
                f"valid: {parsed.is_valid}",
            ]
            display = self._draw_debug_lines(display, lines)

            cv2.imshow("OCR Camera Recognition", display)
            key = cv2.waitKey(1) & 0xFF
            return key not in {27, ord("q"), ord("Q")}
        except cv2.error as exc:
            self._display_disabled = True
            print(f"[ocr-display] OpenCV window disabled: {exc}", file=sys.stderr, flush=True)
            return True

    def _draw_debug_lines(self, frame_bgr: np.ndarray, lines: list[str]) -> np.ndarray:
        image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(image_rgb)
        draw = ImageDraw.Draw(pil_image)
        y = 18
        for line in lines:
            draw.text(
                (12, y),
                line,
                font=self._debug_font,
                fill=(255, 255, 0),
                stroke_width=2,
                stroke_fill=(0, 0, 0),
            )
            y += 34
        return cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

    def _close_debug_window(self) -> None:
        if not self._show_image:
            return
        try:
            cv2.destroyWindow("OCR Camera Recognition")
        except cv2.error:
            return

    def _process_record(self, record) -> None:
        parsed = record.parsed
        if parsed is None:
            return

        output = RecognitionOutput(
            expression=parsed.expression or record.ocr.raw_text,
            answer=parsed.answer,
            answer_mod_4=parsed.answer % 4 if parsed.answer is not None else None,
            is_valid=parsed.is_valid and parsed.answer is not None,
            confidence=record.ocr.confidence,
            backend=record.ocr.backend,
            error=parsed.error or record.ocr.error,
        )
        self._filter.feed(output)
