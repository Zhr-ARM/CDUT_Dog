from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from robocon_ocr.config import PreprocessConfig

_ROW_SMOOTH_WINDOW = 5
_COL_SMOOTH_WINDOW = 7
_DYNAMIC_ENTER_SCALE = 0.18
_DYNAMIC_EXIT_SCALE = 0.55
_FRAME_RATIO_THRESHOLD = 0.75
_FRAME_MIN_CONSECUTIVE = 2
_FULL_WINDOW_REJECT_WIDTH_RATIO = 0.98
_FULL_WINDOW_REJECT_HEIGHT_RATIO = 0.98


@dataclass(slots=True)
class ProjectionSummary:
    row_peak: float
    col_peak: float
    row_enter_threshold: float
    row_exit_threshold: float
    col_enter_threshold: float
    col_exit_threshold: float


@dataclass(slots=True)
class ExpressionRegionResult:
    region_found: bool
    cropped_region: Image.Image | None
    debug_image: Image.Image | None
    bbox: tuple[int, int, int, int] | None
    row_range: tuple[int, int] | None
    col_range: tuple[int, int] | None
    failure_reason: str | None
    projection_summary: ProjectionSummary
    candidate_count: int = 0
    selected_area: float | None = None
    selected_aspect_ratio: float | None = None
    search_window: tuple[int, int, int, int] | None = None
    effective_search_window: tuple[int, int, int, int] | None = None
    otsu_threshold: float | None = None
    mask_debug_image: Image.Image | None = None


def _import_cv2():
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV。请先执行 `pip install -r requirements.txt`。") from exc
    return cv2


def _empty_summary() -> ProjectionSummary:
    return ProjectionSummary(
        row_peak=0.0,
        col_peak=0.0,
        row_enter_threshold=0.0,
        row_exit_threshold=0.0,
        col_enter_threshold=0.0,
        col_exit_threshold=0.0,
    )


def _build_failure_result(
    image: Image.Image,
    failure_reason: str,
    search_window: tuple[int, int, int, int],
    *,
    effective_search_window: tuple[int, int, int, int] | None = None,
    summary: ProjectionSummary | None = None,
    otsu_threshold: float | None = None,
    mask_debug_image: Image.Image | None = None,
) -> ExpressionRegionResult:
    return ExpressionRegionResult(
        region_found=False,
        cropped_region=None,
        debug_image=image,
        bbox=None,
        row_range=None,
        col_range=None,
        failure_reason=failure_reason,
        projection_summary=summary or _empty_summary(),
        candidate_count=0,
        selected_area=None,
        selected_aspect_ratio=None,
        search_window=search_window,
        effective_search_window=effective_search_window,
        otsu_threshold=otsu_threshold,
        mask_debug_image=mask_debug_image,
    )


def _smooth_series(values: np.ndarray, window: int) -> np.ndarray:
    if values.size == 0:
        return values.astype(np.float32)
    width = max(1, min(window, int(values.size)))
    if width <= 1:
        return values.astype(np.float32)
    kernel = np.ones(width, dtype=np.float32) / float(width)
    return np.convolve(values.astype(np.float32), kernel, mode="same")


def _scan_boundary_forward(
    values: np.ndarray,
    *,
    enter_threshold: float,
    exit_threshold: float,
    min_consecutive: int,
) -> int | None:
    state = "background"
    candidate_start: int | None = None
    consecutive = 0

    for index, value in enumerate(values):
        sample = float(value)
        if state == "background":
            if sample >= enter_threshold:
                state = "candidate"
                candidate_start = index
                consecutive = 1
            continue

        if sample >= enter_threshold:
            consecutive += 1
            if consecutive >= min_consecutive:
                return candidate_start
            continue

        if sample <= exit_threshold:
            state = "background"
            candidate_start = None
            consecutive = 0

    return None


def _scan_boundary_backward(
    values: np.ndarray,
    *,
    enter_threshold: float,
    exit_threshold: float,
    min_consecutive: int,
) -> int | None:
    start_in_reverse = _scan_boundary_forward(
        values[::-1],
        enter_threshold=enter_threshold,
        exit_threshold=exit_threshold,
        min_consecutive=min_consecutive,
    )
    if start_in_reverse is None:
        return None
    return len(values) - start_in_reverse


def _clip_bbox(x0: int, y0: int, x1: int, y1: int, width: int, height: int) -> tuple[int, int, int, int]:
    return (
        max(0, min(x0, width)),
        max(0, min(y0, height)),
        max(0, min(x1, width)),
        max(0, min(y1, height)),
    )


def _covers_almost_entire_window(
    bbox: tuple[int, int, int, int],
    effective_search_window: tuple[int, int, int, int],
) -> bool:
    x0, y0, x1, y1 = bbox
    effective_left, effective_top, effective_right, effective_bottom = effective_search_window
    bbox_width = max(0, x1 - x0)
    bbox_height = max(0, y1 - y0)
    window_width = max(1, effective_right - effective_left)
    window_height = max(1, effective_bottom - effective_top)
    return (
        (bbox_width / window_width) >= _FULL_WINDOW_REJECT_WIDTH_RATIO
        and (bbox_height / window_height) >= _FULL_WINDOW_REJECT_HEIGHT_RATIO
    )


def _expand_bbox_by_ratio(
    bbox: tuple[int, int, int, int],
    *,
    expand_ratio_x: float,
    expand_ratio_y: float,
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = bbox
    bbox_width = max(0, x1 - x0)
    bbox_height = max(0, y1 - y0)
    extra_x = int(round(bbox_width * max(0.0, float(expand_ratio_x))))
    extra_y = int(round(bbox_height * max(0.0, float(expand_ratio_y))))
    return _clip_bbox(x0 - extra_x, y0 - extra_y, x1 + extra_x, y1 + extra_y, width, height)


def _apply_otsu(gray: np.ndarray, bias: int) -> tuple[float, np.ndarray]:
    cv2 = _import_cv2()
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    otsu_threshold, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    adjusted_threshold = float(np.clip(otsu_threshold + bias, 0, 255))
    if bias != 0:
        binary = np.where(blurred <= adjusted_threshold, 255, 0).astype(np.uint8)
    return adjusted_threshold, (binary > 0).astype(np.uint8)


def _strip_black_frame(
    binary_black: np.ndarray,
    *,
    border_ratio_threshold: float,
    border_min_consecutive: int,
) -> tuple[int, int, int, int]:
    row_ratios = binary_black.mean(axis=1) if binary_black.size else np.asarray([], dtype=np.float32)
    col_ratios = binary_black.mean(axis=0) if binary_black.size else np.asarray([], dtype=np.float32)

    def _scan_leading(values: np.ndarray) -> int:
        consecutive = 0
        end_index = 0
        for index, value in enumerate(values):
            if float(value) >= border_ratio_threshold:
                consecutive += 1
                end_index = index + 1
                continue
            if consecutive >= border_min_consecutive:
                return end_index
            consecutive = 0
            end_index = 0
        return end_index if consecutive >= border_min_consecutive else 0

    def _scan_trailing(values: np.ndarray) -> int:
        consecutive = 0
        start_index = len(values)
        for index in range(len(values) - 1, -1, -1):
            if float(values[index]) >= border_ratio_threshold:
                consecutive += 1
                start_index = index
                continue
            if consecutive >= border_min_consecutive:
                return start_index
            consecutive = 0
            start_index = len(values)
        return start_index if consecutive >= border_min_consecutive else len(values)

    top_trim = _scan_leading(row_ratios)
    bottom_trim = _scan_trailing(row_ratios)
    left_trim = _scan_leading(col_ratios)
    right_trim = _scan_trailing(col_ratios)
    return left_trim, top_trim, right_trim, bottom_trim


def _dynamic_thresholds(peak: float, config: PreprocessConfig) -> tuple[float, float]:
    enter_threshold = max(float(config.expression_enter_ratio), float(peak) * _DYNAMIC_ENTER_SCALE)
    exit_threshold = max(float(config.expression_exit_ratio), enter_threshold * _DYNAMIC_EXIT_SCALE)
    return enter_threshold, exit_threshold


def _build_mask_debug_image(
    image: Image.Image,
    *,
    search_window: tuple[int, int, int, int],
    effective_search_window: tuple[int, int, int, int],
    effective_mask: np.ndarray,
    bbox: tuple[int, int, int, int] | None,
) -> Image.Image:
    debug_rgb = np.asarray(image.convert("RGB"), dtype=np.uint8).copy()
    x0, y0, x1, y1 = effective_search_window
    full_mask = np.zeros(debug_rgb.shape[:2], dtype=bool)
    full_mask[y0:y1, x0:x1] = effective_mask.astype(bool)
    if np.any(full_mask):
        tinted = debug_rgb[full_mask].astype(np.float32)
        tint_color = np.array([255.0, 72.0, 72.0], dtype=np.float32)
        debug_rgb[full_mask] = np.clip((tinted * 0.55) + (tint_color * 0.45), 0, 255).astype(np.uint8)

    debug_image = Image.fromarray(debug_rgb, mode="RGB")
    draw = ImageDraw.Draw(debug_image)
    draw.rectangle(search_window, outline=(255, 200, 0), width=2)
    draw.rectangle(effective_search_window, outline=(0, 220, 255), width=2)
    if bbox is not None:
        draw.rectangle(bbox, outline=(0, 255, 120), width=2)
    return debug_image


def extract_expression_region(image: Image.Image, config: PreprocessConfig) -> ExpressionRegionResult:
    gray = np.asarray(image.convert("L"), dtype=np.uint8)
    height, width = gray.shape

    top = int(height * config.expression_search_top_ratio)
    bottom = int(height * (1.0 - config.expression_search_bottom_ratio))
    left = int(width * config.expression_search_left_ratio)
    right = int(width * (1.0 - config.expression_search_right_ratio))
    search_window = (left, top, right, bottom)
    if bottom <= top or right <= left:
        return _build_failure_result(image, "search window invalid", search_window)

    search_gray = gray[top:bottom, left:right]
    _, search_mask = _apply_otsu(search_gray, config.expression_otsu_bias)
    frame_left, frame_top, frame_right, frame_bottom = _strip_black_frame(
        search_mask,
        border_ratio_threshold=_FRAME_RATIO_THRESHOLD,
        border_min_consecutive=_FRAME_MIN_CONSECUTIVE,
    )
    if frame_bottom <= frame_top or frame_right <= frame_left:
        return _build_failure_result(image, "frame strip invalid", search_window)

    effective_left = left + frame_left
    effective_top = top + frame_top
    effective_right = left + frame_right
    effective_bottom = top + frame_bottom
    effective_search_window = (effective_left, effective_top, effective_right, effective_bottom)

    effective_gray = search_gray[frame_top:frame_bottom, frame_left:frame_right]
    if effective_gray.size == 0:
        return _build_failure_result(
            image,
            "frame strip invalid",
            search_window,
            effective_search_window=effective_search_window,
        )

    otsu_threshold, effective_mask = _apply_otsu(effective_gray, config.expression_otsu_bias)

    row_values = effective_mask.mean(axis=1) if effective_mask.size else np.asarray([], dtype=np.float32)
    row_ratios = _smooth_series(row_values, _ROW_SMOOTH_WINDOW)
    row_peak = float(row_ratios.max()) if row_ratios.size else 0.0
    row_enter_threshold, row_exit_threshold = _dynamic_thresholds(row_peak, config)

    top_boundary = _scan_boundary_forward(
        row_ratios,
        enter_threshold=row_enter_threshold,
        exit_threshold=row_exit_threshold,
        min_consecutive=max(1, int(config.expression_min_consecutive_rows)),
    )
    bottom_boundary = _scan_boundary_backward(
        row_ratios,
        enter_threshold=row_enter_threshold,
        exit_threshold=row_exit_threshold,
        min_consecutive=max(1, int(config.expression_min_consecutive_rows)),
    )

    summary = ProjectionSummary(
        row_peak=row_peak,
        col_peak=0.0,
        row_enter_threshold=row_enter_threshold,
        row_exit_threshold=row_exit_threshold,
        col_enter_threshold=0.0,
        col_exit_threshold=0.0,
    )
    if top_boundary is None:
        return _build_failure_result(
            image,
            "row top not found",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )
    if bottom_boundary is None:
        return _build_failure_result(
            image,
            "row bottom not found",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )
    if bottom_boundary <= top_boundary:
        return _build_failure_result(
            image,
            "region too small",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )

    inner_mask = effective_mask[top_boundary:bottom_boundary, :]
    col_values = inner_mask.mean(axis=0) if inner_mask.size else np.asarray([], dtype=np.float32)
    col_ratios = _smooth_series(col_values, _COL_SMOOTH_WINDOW)
    col_peak = float(col_ratios.max()) if col_ratios.size else 0.0
    col_enter_threshold, col_exit_threshold = _dynamic_thresholds(col_peak, config)
    summary = ProjectionSummary(
        row_peak=row_peak,
        col_peak=col_peak,
        row_enter_threshold=row_enter_threshold,
        row_exit_threshold=row_exit_threshold,
        col_enter_threshold=col_enter_threshold,
        col_exit_threshold=col_exit_threshold,
    )

    left_boundary = _scan_boundary_forward(
        col_ratios,
        enter_threshold=col_enter_threshold,
        exit_threshold=col_exit_threshold,
        min_consecutive=max(1, int(config.expression_min_consecutive_cols)),
    )
    right_boundary = _scan_boundary_backward(
        col_ratios,
        enter_threshold=col_enter_threshold,
        exit_threshold=col_exit_threshold,
        min_consecutive=max(1, int(config.expression_min_consecutive_cols)),
    )
    if left_boundary is None:
        return _build_failure_result(
            image,
            "column left not found",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )
    if right_boundary is None:
        return _build_failure_result(
            image,
            "column right not found",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )
    if right_boundary <= left_boundary:
        return _build_failure_result(
            image,
            "region too small",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )

    x0, y0, x1, y1 = _clip_bbox(
        effective_left + left_boundary - config.expression_bbox_padding_x,
        effective_top + top_boundary - config.expression_bbox_padding_y,
        effective_left + right_boundary + config.expression_bbox_padding_x,
        effective_top + bottom_boundary + config.expression_bbox_padding_y,
        width,
        height,
    )
    if x1 <= x0 or y1 <= y0:
        return _build_failure_result(
            image,
            "region too small",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=None,
            ),
        )

    bbox = _expand_bbox_by_ratio(
        (x0, y0, x1, y1),
        expand_ratio_x=config.expression_bbox_expand_ratio_x,
        expand_ratio_y=config.expression_bbox_expand_ratio_y,
        width=width,
        height=height,
    )
    cropped = image.crop(bbox)
    if _covers_almost_entire_window(bbox, effective_search_window):
        return _build_failure_result(
            image,
            "region too small",
            search_window,
            effective_search_window=effective_search_window,
            summary=summary,
            otsu_threshold=otsu_threshold,
            mask_debug_image=_build_mask_debug_image(
                image,
                search_window=search_window,
                effective_search_window=effective_search_window,
                effective_mask=effective_mask,
                bbox=bbox,
            ),
        )

    area = float((x1 - x0) * (y1 - y0))
    aspect_ratio = float((x1 - x0) / max(y1 - y0, 1))
    mask_debug_image = _build_mask_debug_image(
        image,
        search_window=search_window,
        effective_search_window=effective_search_window,
        effective_mask=effective_mask,
        bbox=bbox,
    )
    return ExpressionRegionResult(
        region_found=True,
        cropped_region=cropped,
        debug_image=cropped,
        bbox=bbox,
        row_range=(y0, y1),
        col_range=(x0, x1),
        failure_reason=None,
        projection_summary=summary,
        candidate_count=1,
        selected_area=area,
        selected_aspect_ratio=aspect_ratio,
        search_window=search_window,
        effective_search_window=effective_search_window,
        otsu_threshold=otsu_threshold,
        mask_debug_image=mask_debug_image,
    )
