from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from functools import lru_cache
import os
from pathlib import Path
import sys
import threading
import time
from time import strftime

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from robocon_ocr.camera_session import LatestFrameBuffer
from robocon_ocr.camera_tuning import DEFAULT_CAMERA_TUNING
from robocon_ocr.config import CameraConfig, OCRConfig, PipelineConfig
from robocon_ocr.image_recognition.factory import create_recognizer
from robocon_ocr.result.reporter import PipelineRecord, summarize
from robocon_ocr.staged_pipeline import STAGE_SEQUENCE
from robocon_ocr.staged_pipeline import PipelineContext
from robocon_ocr.staged_pipeline import context_to_record
from robocon_ocr.staged_pipeline import run_camera_pipeline_frame
from robocon_ocr.staged_pipeline import save_stage_debug_images
from robocon_ocr.vision_capture.usb_camera import USBCameraCapture
from robocon_ocr.vision_processing.board_detection import ROIDebugInfo
from robocon_ocr.pipeline import run_pipeline

STAGE_CHOICES = list(STAGE_SEQUENCE)
OCR_BACKEND_CHOICES = ["api", "onnx"]


def _load_dotenv() -> None:
    """加载项目根目录的 .env 文件（如果存在）。

    仅在当前进程未设置对应变量时才覆盖，命令行参数优先级更高。
    """
    env_path = Path(__file__).resolve().parent.parent / ".env"
    if not env_path.is_file():
        return
    with env_path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            key, value = key.strip(), value.strip()
            if key and key not in os.environ:
                os.environ[key] = value


@dataclass(slots=True)
class CameraStats:
    frames_captured: int = 0
    frames_processed: int = 0
    frames_skipped_no_roi: int = 0
    frames_dropped_stale: int = 0
    frames_emitted: int = 0


@dataclass(slots=True)
class DisplayState:
    context: PipelineContext
    record: PipelineRecord
    frame_index: int
    stage_ms: float


def _add_stage_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--stop-after-stage",
        choices=STAGE_CHOICES,
        help="Stop the pipeline after a specific stage.",
    )
    parser.add_argument(
        "--debug-save-stages",
        action="store_true",
        help="Save stage-by-stage debug images into --debug-dir.",
    )
    parser.add_argument(
        "--ocr-backend",
        choices=OCR_BACKEND_CHOICES,
        help="OCR backend override. Both dataset and camera default to api.",
    )


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run Robocon OCR on datasets or a USB camera frame.")
    subparsers = parser.add_subparsers(dest="command")

    dataset_parser = subparsers.add_parser("dataset", help="Run OCR on a dataset directory.")
    dataset_parser.add_argument("dataset_dir", type=Path, help="Dataset image directory.")
    dataset_parser.add_argument(
        "--label-file",
        type=Path,
        help="Tab-separated label file. Defaults to <dataset_dir>/problems_and_answers.txt if present.",
    )
    dataset_parser.add_argument(
        "--debug-dir",
        type=Path,
        help="Directory for cropped/preprocessed debug images.",
    )
    _add_stage_arguments(dataset_parser)

    camera_parser = subparsers.add_parser("camera", help="Run realtime OCR on a USB camera stream.")
    camera_parser.add_argument("--device-index", type=int, help="USB camera device index. Default comes from camera_tuning.py.")
    camera_parser.add_argument("--width", type=int, help="Capture width. Default comes from camera_tuning.py.")
    camera_parser.add_argument("--height", type=int, help="Capture height. Default comes from camera_tuning.py.")
    camera_parser.add_argument("--fps", type=float, help="Requested capture FPS. Default comes from camera_tuning.py.")
    camera_parser.add_argument(
        "--pixel-format",
        help="Requested fourcc pixel format such as MJPG. Default comes from camera_tuning.py.",
    )
    camera_parser.add_argument(
        "--warmup-frames",
        type=int,
        help="Number of frames to discard before OCR. Default comes from camera_tuning.py.",
    )
    camera_parser.add_argument(
        "--capture-timeout-ms",
        type=int,
        help="Camera capture timeout in milliseconds. Default comes from camera_tuning.py.",
    )
    camera_parser.add_argument(
        "--interval-ms",
        type=int,
        help="Legacy delay between OCR attempts in milliseconds. Default comes from camera_tuning.py.",
    )
    camera_parser.add_argument(
        "--max-frames",
        type=int,
        help="Optional maximum number of captured frames before exit.",
    )
    camera_parser.add_argument(
        "--print-all",
        action="store_true",
        help="Print every OCR result instead of only changes.",
    )
    camera_parser.add_argument(
        "--show-window",
        action="store_true",
        help="Show cv2 debug windows with the live frame and OCR details.",
    )
    camera_parser.add_argument(
        "--show-stage-debug",
        action="store_true",
        help="Render the stage dashboard in the debug window.",
    )
    camera_parser.add_argument(
        "--stage-panel-layout",
        default="auto",
        choices=["auto"],
        help="Stage panel layout preset. Current supported value: auto.",
    )
    camera_parser.add_argument(
        "--window-scale",
        type=float,
        default=0.75,
        help="Scale factor for cv2 debug windows, default: 0.75",
    )
    camera_parser.add_argument(
        "--save-frame",
        type=Path,
        help="Optional path to save the captured RGB frame before OCR.",
    )
    camera_parser.add_argument(
        "--debug-dir",
        type=Path,
        help="Directory for cropped/preprocessed debug images.",
    )
    _add_stage_arguments(camera_parser)
    return parser


def resolve_label_file(dataset_dir: Path, label_file: Path | None) -> Path | None:
    if label_file is not None:
        return label_file.expanduser()

    candidate = dataset_dir / "problems_and_answers.txt"
    if candidate.is_file():
        return candidate
    return None


def build_config(args: argparse.Namespace) -> PipelineConfig:
    dataset_dir = args.dataset_dir.expanduser()
    return PipelineConfig(
        dataset_dir=dataset_dir,
        label_file=resolve_label_file(dataset_dir, args.label_file),
        debug_dir=args.debug_dir.expanduser() if args.debug_dir else None,
        stop_after_stage=args.stop_after_stage,
        debug_save_stages=args.debug_save_stages,
        ocr=OCRConfig(backend=args.ocr_backend or "api"),
    )


def build_camera_pipeline_config(args: argparse.Namespace) -> PipelineConfig:
    return PipelineConfig(
        dataset_dir=Path("."),
        debug_dir=args.debug_dir.expanduser() if args.debug_dir else None,
        stop_after_stage=args.stop_after_stage,
        debug_save_stages=args.debug_save_stages,
        ocr=OCRConfig(backend=args.ocr_backend or "api"),
    )


def build_camera_config(args: argparse.Namespace) -> CameraConfig:
    defaults = DEFAULT_CAMERA_TUNING
    return CameraConfig(
        device_index=args.device_index if args.device_index is not None else defaults.device_index,
        width=args.width if args.width is not None else defaults.width,
        height=args.height if args.height is not None else defaults.height,
        fps=args.fps if args.fps is not None else defaults.fps,
        pixel_format=args.pixel_format if args.pixel_format is not None else defaults.pixel_format,
        warmup_frames=args.warmup_frames if args.warmup_frames is not None else defaults.warmup_frames,
        capture_timeout_ms=(
            args.capture_timeout_ms if args.capture_timeout_ms is not None else defaults.capture_timeout_ms
        ),
        interval_ms=args.interval_ms if args.interval_ms is not None else defaults.interval_ms,
        max_frames=args.max_frames,
        emit_only_changes=not args.print_all,
        save_frame=args.save_frame.expanduser() if args.save_frame else None,
        async_latest_frame=defaults.async_latest_frame,
        auto_exposure=defaults.auto_exposure,
        exposure_dynamic_framerate=defaults.exposure_dynamic_framerate,
        exposure_time_absolute=defaults.exposure_time_absolute,
        gain=defaults.gain,
        brightness=defaults.brightness,
        contrast=defaults.contrast,
        saturation=defaults.saturation,
        sharpness=defaults.sharpness,
        gamma=defaults.gamma,
        white_balance_automatic=defaults.white_balance_automatic,
        white_balance_temperature=defaults.white_balance_temperature,
        focus_automatic_continuous=defaults.focus_automatic_continuous,
        focus_absolute=defaults.focus_absolute,
        backlight_compensation=defaults.backlight_compensation,
        power_line_frequency=defaults.power_line_frequency,
    )


def print_records(records) -> None:
    for record in records:
        print(f"[{record.image_name}]")
        print(f"  ocr_backend: {record.ocr.backend}")
        print(f"  raw_text: {record.ocr.raw_text}")
        print(f"  normalized: {record.parsed.normalized_text}")
        print(f"  expression: {record.parsed.expression}")
        print(f"  answer: {record.parsed.answer}")
        print(f"  confidence: {record.ocr.confidence:.4f}")
        print(f"  psm: {record.ocr.psm}")
        print(f"  valid: {record.parsed.is_valid}")
        print(f"  roi_found: {record.roi_found}")
        if record.roi_quad is not None:
            print(f"  roi_quad: {record.roi_quad}")
        if record.label is not None:
            print(f"  gt_expression: {record.label.expression}")
            print(f"  gt_answer: {record.label.answer}")
            print(f"  expression_match: {record.expression_match}")
            print(f"  answer_match: {record.answer_match}")
        if record.ocr.error:
            print(f"  ocr_error: {record.ocr.error}")
        if record.parsed.error:
            print(f"  error: {record.parsed.error}")


def _camera_signature(record: PipelineRecord) -> tuple[str, int | None, bool, str | None]:
    return (
        record.parsed.expression,
        record.parsed.answer,
        record.parsed.is_valid,
        record.parsed.error,
    )


def print_camera_record(record: PipelineRecord, frame_index: int) -> None:
    timestamp = strftime("%H:%M:%S")
    print(f"[camera frame={frame_index} time={timestamp}]")
    print_records([record])
    print()


def _wrap_text(text: str, width: int = 48) -> list[str]:
    if not text:
        return [""]
    lines: list[str] = []
    remaining = text
    while len(remaining) > width:
        lines.append(remaining[:width])
        remaining = remaining[width:]
    lines.append(remaining)
    return lines


def _draw_roi_quad(frame_bgr: np.ndarray, roi_quad) -> np.ndarray:
    if roi_quad is None:
        return frame_bgr
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    canvas = frame_bgr.copy()
    points = np.array(roi_quad, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(canvas, [points], isClosed=True, color=(0, 255, 255), thickness=3)
    return canvas


def _format_roi_debug_lines(roi_debug: ROIDebugInfo) -> list[str]:
    def fmt(value, digits: int = 3) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, int):
            return str(value)
        return f"{value:.{digits}f}"

    candidate_label = roi_debug.best_candidate_type or "none"
    reason = roi_debug.failure_reason or "passed"
    tolerance_label = (
        f"RectTol: {fmt(roi_debug.rectangle_ratio_tolerance)}"
        if candidate_label == "rectangle"
        else f"QuadTol: {fmt(roi_debug.quadrilateral_ratio_tolerance)}"
    )
    return [
        f"ROI: {roi_debug.roi_found}",
        f"Reason: {reason}",
        f"Source: {roi_debug.best_candidate_source}",
        f"Type: {candidate_label}",
        f"Area: {fmt(roi_debug.best_candidate_area_ratio)} / {fmt(roi_debug.min_area_ratio_threshold)}",
        f"CompArea: {fmt(roi_debug.best_candidate_component_area_ratio)}",
        f"Fill: {fmt(roi_debug.best_candidate_rect_fill_ratio)}",
        f"Edge: {fmt(roi_debug.best_candidate_edge_strength, 1)} / {fmt(roi_debug.edge_threshold, 1)}",
        f"Ratio: {fmt(roi_debug.best_candidate_ratio)}",
        f"Err: {fmt(roi_debug.best_candidate_ratio_error)}",
        f"Corner: {roi_debug.corner_found}",
        f"CompCnt: {roi_debug.component_count}",
        f"CompRank: {fmt(roi_debug.component_rank, 0)}",
        tolerance_label,
        f"WhiteThr: {fmt(roi_debug.white_threshold, 0)}",
        f"Cand: {roi_debug.candidate_count}",
    ]


def _put_panel_title(image_bgr: np.ndarray, title: str) -> np.ndarray:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    canvas = image_bgr.copy()
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (20, 20, 20), thickness=-1)
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (235, 235, 235), 2, cv2.LINE_AA)
    return canvas


def _fit_panel(image_bgr: np.ndarray, target_size: tuple[int, int]) -> np.ndarray:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    target_w, target_h = target_size
    return cv2.resize(image_bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)


def _fit_panel_preserve_aspect(image_bgr: np.ndarray, target_size: tuple[int, int]) -> np.ndarray:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    target_w, target_h = target_size
    src_h, src_w = image_bgr.shape[:2]
    if src_h <= 0 or src_w <= 0:
        return np.zeros((target_h, target_w, 3), dtype=np.uint8)

    scale = min(target_w / src_w, target_h / src_h)
    resized_w = max(1, int(round(src_w * scale)))
    resized_h = max(1, int(round(src_h * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image_bgr, (resized_w, resized_h), interpolation=interpolation)

    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    offset_x = (target_w - resized_w) // 2
    offset_y = (target_h - resized_h) // 2
    canvas[offset_y : offset_y + resized_h, offset_x : offset_x + resized_w] = resized
    return canvas


def _pil_to_bgr(image: Image.Image | None, target_size: tuple[int, int]) -> np.ndarray:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    width, height = target_size
    if image is None:
        return np.zeros((height, width, 3), dtype=np.uint8)
    rgb = image.convert("RGB")
    bgr = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
    return _fit_panel_preserve_aspect(bgr, target_size)


def _build_info_lines(display: DisplayState) -> list[str]:
    context = display.context
    record = display.record
    lines = [
        "Pipeline Status",
        f"Frame: {display.frame_index}",
        f"Latency: {display.stage_ms:.1f} ms",
        f"Completed: {context.completed_stage or 'none'}",
        f"StopAfter: {context.stop_after_stage or 'none'}",
        f"OCR Backend: {record.ocr.backend}",
        f"ROI: {record.roi_found}",
        f"Confidence: {record.ocr.confidence:.4f}",
        f"Valid: {record.parsed.is_valid}",
        "Raw:",
    ]
    lines.extend(_wrap_text(record.ocr.raw_text or "<empty>"))
    lines.append("Expression:")
    lines.extend(_wrap_text(record.parsed.expression or "<empty>"))
    lines.append(f"Answer: {record.parsed.answer}")
    if record.ocr.error:
        lines.append("OCR Error:")
        lines.extend(_wrap_text(record.ocr.error))
    if record.parsed.error:
        lines.append("Error:")
        lines.extend(_wrap_text(record.parsed.error))
    if context.board_detection is not None:
        lines.append("Board Debug:")
        lines.extend(_format_roi_debug_lines(context.board_detection.roi_debug)[:6])
    if context.expression_region is not None:
        lines.append("Expr Region:")
        lines.append(f"Found: {context.expression_region.region_found}")
        lines.append(f"BBox: {context.expression_region.bbox}")
        lines.append(f"Rows: {context.expression_region.row_range}")
        lines.append(f"Cols: {context.expression_region.col_range}")
        lines.append(f"Otsu: {context.expression_region.otsu_threshold}")
        lines.append(f"RowPeak: {context.expression_region.projection_summary.row_peak:.4f}")
        lines.append(
            "RowThr: "
            f"{context.expression_region.projection_summary.row_enter_threshold:.4f}/"
            f"{context.expression_region.projection_summary.row_exit_threshold:.4f}"
        )
        lines.append(f"ColPeak: {context.expression_region.projection_summary.col_peak:.4f}")
        lines.append(
            "ColThr: "
            f"{context.expression_region.projection_summary.col_enter_threshold:.4f}/"
            f"{context.expression_region.projection_summary.col_exit_threshold:.4f}"
        )
        lines.append(f"Search: {context.expression_region.search_window}")
        lines.append(f"Effective: {context.expression_region.effective_search_window}")
        if context.expression_region.failure_reason:
            lines.append(f"Reason: {context.expression_region.failure_reason}")
    snapshot = context.snapshots.get(context.completed_stage or "")
    if snapshot is not None and snapshot.lines:
        lines.append("Stage Notes:")
        lines.extend(snapshot.lines)
    return lines


def _build_text_panel(lines: list[str], panel_size: tuple[int, int]) -> np.ndarray:
    panel_w, panel_h = panel_size
    image = Image.new("RGB", (panel_w, panel_h), (28, 28, 28))
    draw = ImageDraw.Draw(image)
    text_x = 18
    y = 42
    title_font = _load_dashboard_font(24)
    body_font = _load_dashboard_font(18)

    for index, line in enumerate(lines):
        if y >= panel_h - 10:
            break
        font = title_font if index == 0 else body_font
        color = (120, 220, 255) if index == 0 else (235, 235, 235)
        if line.startswith("Error:") or line.startswith("OCR Error:") or line.startswith("Reason:"):
            color = (80, 160, 255)
        draw.text((text_x, y - 18), line, font=font, fill=color)
        y += 30 if index == 0 else 22
    return np.asarray(image)[:, :, ::-1].copy()


def _build_status_panel(display: DisplayState, panel_size: tuple[int, int]) -> np.ndarray:
    panel_w, panel_h = panel_size
    image = Image.new("RGB", (panel_w, panel_h), (28, 28, 28))
    draw = ImageDraw.Draw(image)

    result_box = (18, 42, panel_w - 18, min(172, panel_h - 18))
    draw.rounded_rectangle(result_box, radius=16, fill=(18, 52, 30), outline=(64, 170, 104), width=2)

    title_font = _load_dashboard_font(24)
    result_font = _load_dashboard_font(30)
    answer_font = _load_dashboard_font(34)
    body_font = _load_dashboard_font(18)

    expression = display.record.parsed.expression or display.record.ocr.raw_text or "<empty>"
    answer = str(display.record.parsed.answer) if display.record.parsed.answer is not None else "<pending>"
    draw.text((34, 56), "Recognized Expression", font=title_font, fill=(188, 255, 208))
    draw.text((34, 88), expression, font=result_font, fill=(92, 255, 138))
    draw.text((34, 126), f"Answer: {answer}", font=answer_font, fill=(124, 255, 156))

    lines = _build_info_lines(display)
    text_x = 18
    y = result_box[3] + 28
    for index, line in enumerate(lines):
        if y >= panel_h - 10:
            break
        font = title_font if index == 0 else body_font
        color = (120, 220, 255) if index == 0 else (235, 235, 235)
        if line.startswith("Error:") or line.startswith("OCR Error:") or line.startswith("Reason:"):
            color = (80, 160, 255)
        draw.text((text_x, y - 18), line, font=font, fill=color)
        y += 30 if index == 0 else 22

    return np.asarray(image)[:, :, ::-1].copy()


@lru_cache(maxsize=8)
def _load_dashboard_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    )
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def _show_debug_windows(display: DisplayState, window_scale: float) -> bool:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV GUI 版本，无法显示调试窗口。") from exc

    try:
        original_bgr = cv2.cvtColor(np.asarray(display.context.original), cv2.COLOR_RGB2BGR)
        original_bgr = _draw_roi_quad(original_bgr, display.record.roi_quad)
        panel_w = original_bgr.shape[1]
        panel_h = original_bgr.shape[0]

        board_debug = None
        if display.context.board_detection is not None:
            board_debug = display.context.board_detection.debug_image
        stage_snapshot = display.context.snapshots.get(display.context.completed_stage or "")
        stage_image = None if stage_snapshot is None else stage_snapshot.image
        info_lines = _build_info_lines(display)

        top_left = _put_panel_title(_fit_panel_preserve_aspect(original_bgr, (panel_w, panel_h)), "Color Original")
        top_right = _put_panel_title(_pil_to_bgr(board_debug, (panel_w, panel_h)), "Board Detection")
        bottom_left = _put_panel_title(_pil_to_bgr(stage_image, (panel_w, panel_h)), "Current Stage Output")
        bottom_right = _put_panel_title(
            _fit_panel(_build_status_panel(display, (panel_w, panel_h)), (panel_w, panel_h)),
            "Stage / OCR Status",
        )

        top_row = np.hstack([top_left, top_right])
        bottom_row = np.hstack([bottom_left, bottom_right])
        dashboard = np.vstack([top_row, bottom_row])
        if window_scale != 1.0:
            dashboard = cv2.resize(dashboard, None, fx=window_scale, fy=window_scale, interpolation=cv2.INTER_AREA)

        cv2.imshow("robocon_ocr_dashboard", dashboard)
        key = cv2.waitKey(1) & 0xFF
        return key not in {27, ord("q"), ord("Q")}
    except cv2.error as exc:
        raise RuntimeError(
            "当前 OpenCV 构建不支持窗口显示。你的环境里很可能装了 `opencv-python-headless`。"
            "请先卸载 `opencv-python-headless`，再重装带 GUI 的 `opencv-python`；"
            "如果只是想跑识别，不开窗口，可以去掉 `--show-window`。"
        ) from exc


def _save_camera_frame(image_rgb: Image.Image, save_path: Path | None) -> None:
    if save_path is None:
        return
    save_path.parent.mkdir(parents=True, exist_ok=True)
    image_rgb.save(save_path)


def _save_camera_debug_outputs(context: PipelineContext, pipeline_config: PipelineConfig) -> None:
    if pipeline_config.debug_dir is None:
        return
    pipeline_config.debug_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(context.image_name).stem
    if context.rectification is not None:
        context.rectification.cropped.save(pipeline_config.debug_dir / f"{stem}_cropped.png")
        context.rectification.rectified.save(pipeline_config.debug_dir / f"{stem}_rectified.png")
    if context.enhancement is not None:
        context.enhancement.prepared_for_ocr.save(pipeline_config.debug_dir / f"{stem}_prepared.png")
    if context.expression_region is not None and context.expression_region.cropped_region is not None:
        context.expression_region.cropped_region.save(pipeline_config.debug_dir / f"{stem}_expression_region.png")
    if pipeline_config.debug_save_stages:
        save_stage_debug_images(context, pipeline_config.debug_dir)


def _process_camera_frame(
    frame_bgr: np.ndarray,
    frame_index: int,
    args: argparse.Namespace,
    pipeline_config: PipelineConfig,
    recognizer,
) -> tuple[PipelineRecord, PipelineContext, float]:
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV。请先执行 `pip install -r requirements.txt`。") from exc

    image_name = f"camera_{args.device_index}_{frame_index:06d}.png"
    image_rgb = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    started_at = time.perf_counter()
    context = run_camera_pipeline_frame(
        frame=image_rgb,
        image_name=image_name,
        config=pipeline_config,
        recognizer=recognizer,
    )
    _save_camera_frame(image_rgb, args.save_frame.expanduser() if args.save_frame else None)
    _save_camera_debug_outputs(context, pipeline_config)
    record = context_to_record(context)
    return record, context, (time.perf_counter() - started_at) * 1000.0


def _stop_before_ocr(stop_after_stage: str | None) -> bool:
    if stop_after_stage is None:
        return False
    return STAGE_SEQUENCE.index(stop_after_stage) < STAGE_SEQUENCE.index("ocr")


def _run_async_camera(args: argparse.Namespace) -> int:
    camera = USBCameraCapture(build_camera_config(args))
    pipeline_config = build_camera_pipeline_config(args)
    recognizer = create_recognizer(pipeline_config.ocr)
    if pipeline_config.ocr.warmup and not _stop_before_ocr(pipeline_config.stop_after_stage):
        recognizer.warmup()
    buffer = LatestFrameBuffer()
    stats = CameraStats()
    stop_event = threading.Event()
    display_lock = threading.Lock()
    display_state: DisplayState | None = None
    last_signature = None
    last_processed_index = -1
    processing_error: Exception | None = None

    def capture_worker() -> None:
        try:
            for frame_index, frame_bgr in camera.stream_raw_frames():
                if stop_event.is_set():
                    break
                stats.frames_captured += 1
                buffer.publish(frame_index, frame_bgr)
        except Exception as exc:
            buffer.stop(exc)
        else:
            buffer.stop()

    def ocr_worker() -> None:
        nonlocal display_state, last_signature, last_processed_index, processing_error
        try:
            while not stop_event.is_set():
                item = buffer.wait_for_next(last_processed_index)
                if item is None:
                    break
                frame_index, frame_bgr, _captured_at = item
                if last_processed_index >= 0 and frame_index > last_processed_index + 1:
                    stats.frames_dropped_stale += frame_index - last_processed_index - 1
                record, context, stage_ms = _process_camera_frame(
                    frame_bgr=frame_bgr,
                    frame_index=frame_index,
                    args=args,
                    pipeline_config=pipeline_config,
                    recognizer=recognizer,
                )
                last_processed_index = frame_index
                is_pending = record.ocr.error == "pending"
                if not record.roi_found:
                    stats.frames_skipped_no_roi += 1
                elif is_pending:
                    stats.frames_processed += 1
                else:
                    stats.frames_processed += 1
                    signature = _camera_signature(record)
                    if args.print_all or signature != last_signature:
                        print_camera_record(record, frame_index)
                        last_signature = signature
                        stats.frames_emitted += 1
                with display_lock:
                    display_state = DisplayState(
                        context=context,
                        record=record,
                        frame_index=frame_index,
                        stage_ms=stage_ms,
                    )
        except Exception as exc:
            processing_error = exc
            stop_event.set()
            buffer.stop(exc)

    capture_thread = threading.Thread(target=capture_worker, name="camera-capture", daemon=True)
    ocr_thread = threading.Thread(target=ocr_worker, name="camera-ocr", daemon=True)
    capture_thread.start()
    ocr_thread.start()

    try:
        if args.show_window:
            while capture_thread.is_alive() or ocr_thread.is_alive():
                with display_lock:
                    current_display = display_state
                if current_display is not None:
                    keep_running = _show_debug_windows(current_display, args.window_scale)
                    if not keep_running:
                        stop_event.set()
                        buffer.stop()
                        break
                else:
                    time.sleep(0.01)
        capture_thread.join()
        ocr_thread.join()
    except KeyboardInterrupt:
        stop_event.set()
        buffer.stop()
        capture_thread.join()
        ocr_thread.join()
        print("\n[camera] stopped by user")
    finally:
        if args.show_window:
            try:
                import cv2

                cv2.destroyAllWindows()
            except Exception:
                pass

    if processing_error is not None:
        raise processing_error

    print("[summary]")
    print(f"  frames_captured: {stats.frames_captured}")
    print(f"  frames_processed: {stats.frames_processed}")
    print(f"  frames_skipped_no_roi: {stats.frames_skipped_no_roi}")
    print(f"  frames_dropped_stale: {stats.frames_dropped_stale}")
    print(f"  frames_emitted: {stats.frames_emitted}")
    return 0


def main(argv: list[str] | None = None) -> int:
    _load_dotenv()
    args_list = list(argv) if argv is not None else sys.argv[1:]
    if args_list and args_list[0] not in {"dataset", "camera", "-h", "--help"}:
        args_list = ["dataset", *args_list]

    args = build_argparser().parse_args(args_list)

    if args.command in {None, "dataset"}:
        config = build_config(args)
        records = run_pipeline(config)
        print_records(records)
        summary = summarize(records)
        print("[summary]")
        for key, value in summary.items():
            print(f"  {key}: {value}")
        return 0

    return _run_async_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
