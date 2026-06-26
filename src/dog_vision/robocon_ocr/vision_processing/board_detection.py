from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

from robocon_ocr.config import PreprocessConfig


@dataclass(slots=True)
class ROIDebugInfo:
    roi_found: bool
    failure_reason: str | None
    best_candidate_source: str
    best_candidate_type: str
    best_candidate_area_ratio: float | None
    best_candidate_component_area_ratio: float | None
    best_candidate_rect_fill_ratio: float | None
    best_candidate_edge_strength: float | None
    best_candidate_ratio: float | None
    best_candidate_ratio_error: float | None
    component_count: int
    component_rank: int | None
    corner_found: bool
    min_area_ratio_threshold: float
    edge_threshold: float
    rectangle_ratio_tolerance: float
    quadrilateral_ratio_tolerance: float
    white_threshold: float
    candidate_count: int


@dataclass(slots=True)
class BoardDetectionResult:
    roi_found: bool
    roi_quad: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]] | None
    ordered_quad: np.ndarray | None
    board_binary: Image.Image
    debug_image: Image.Image
    roi_debug: ROIDebugInfo
    candidate_source: str
    candidate_type: str
    candidate_count: int
    component_count: int


@dataclass(slots=True)
class _RectangleCandidate:
    quad: np.ndarray
    area_ratio: float
    fill_ratio: float
    edge_strength: float
    ratio: float
    ratio_error: float
    right_angle_error: float
    score: float


def _import_cv2():
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV。请先执行 `pip install -r requirements.txt`。") from exc
    return cv2


def order_quad(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape != (4, 2):
        raise ValueError("roi quad must contain exactly four points")

    ordered = np.zeros((4, 2), dtype=np.float32)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    ordered[0] = pts[np.argmin(sums)]
    ordered[2] = pts[np.argmax(sums)]
    ordered[1] = pts[np.argmin(diffs)]
    ordered[3] = pts[np.argmax(diffs)]
    return ordered


def clip_quad(points: np.ndarray, width: int, height: int) -> np.ndarray:
    clipped = np.asarray(points, dtype=np.float32).copy()
    clipped[:, 0] = np.clip(clipped[:, 0], 0, width - 1)
    clipped[:, 1] = np.clip(clipped[:, 1], 0, height - 1)
    return clipped


def expand_quad(points: np.ndarray, padding: int, width: int, height: int) -> np.ndarray:
    if padding == 0:
        return clip_quad(points, width, height)

    center = points.mean(axis=0)
    vectors = points - center
    lengths = np.linalg.norm(vectors, axis=1, keepdims=True)
    safe_lengths = np.where(lengths == 0, 1.0, lengths)
    expanded = points + (vectors / safe_lengths) * float(padding)
    return clip_quad(expanded, width, height)


def _quad_aspect_ratio(points: np.ndarray) -> float:
    ordered = order_quad(points)
    top = np.linalg.norm(ordered[1] - ordered[0])
    bottom = np.linalg.norm(ordered[2] - ordered[3])
    left = np.linalg.norm(ordered[3] - ordered[0])
    right = np.linalg.norm(ordered[2] - ordered[1])
    width = max((top + bottom) * 0.5, 1.0)
    height = max((left + right) * 0.5, 1.0)
    return width / height


def _ratio_error(ratio: float, target: float) -> float:
    return abs(ratio - target) / max(target, 1e-6)


def _cosine_between(v1: np.ndarray, v2: np.ndarray) -> float:
    denom = np.linalg.norm(v1) * np.linalg.norm(v2)
    if denom <= 1e-6:
        return 1.0
    return float(np.dot(v1, v2) / denom)


def _right_angle_error(quad: np.ndarray) -> float:
    ordered = order_quad(quad)
    errors: list[float] = []
    for index in range(4):
        prev_pt = ordered[(index - 1) % 4]
        curr_pt = ordered[index]
        next_pt = ordered[(index + 1) % 4]
        v1 = prev_pt - curr_pt
        v2 = next_pt - curr_pt
        errors.append(abs(_cosine_between(v1, v2)))
    return max(errors) if errors else 1.0


def _build_board_binary(gray: np.ndarray, config: PreprocessConfig) -> np.ndarray:
    cv2 = _import_cv2()
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bright = cv2.threshold(blurred, config.white_threshold, 255, cv2.THRESH_BINARY)
    low = max(10, int(config.edge_threshold * 0.5))
    high = max(low + 10, int(config.edge_threshold * 1.6))
    edges = cv2.Canny(blurred, low, high)
    edges = cv2.bitwise_or(edges, bright)
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel, iterations=1)
    return edges


def _contour_edge_strength(edge_binary: np.ndarray, contour: np.ndarray) -> float:
    cv2 = _import_cv2()
    contour_mask = np.zeros_like(edge_binary)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=2)
    return float(cv2.mean(edge_binary, mask=contour_mask)[0])


def _rectangle_score(area_ratio: float, fill_ratio: float, ratio_error: float, right_angle_error: float) -> float:
    area_part = area_ratio * 3.0
    fill_part = fill_ratio * 1.6
    ratio_part = max(0.0, 1.0 - ratio_error)
    angle_part = max(0.0, 1.0 - (right_angle_error * 3.0))
    return area_part + fill_part + ratio_part + angle_part


def _extract_rectangle_candidate(
    contour: np.ndarray,
    edge_binary: np.ndarray,
    frame_area: float,
    config: PreprocessConfig,
) -> _RectangleCandidate | None:
    cv2 = _import_cv2()
    area = cv2.contourArea(contour)
    if area <= 0:
        return None
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return None

    epsilon = 0.02 * perimeter
    approx = cv2.approxPolyDP(contour, epsilon, True)
    if len(approx) != 4:
        return None
    if not cv2.isContourConvex(approx):
        return None

    quad = approx.reshape(4, 2).astype(np.float32)
    ordered = order_quad(quad)
    quad_area = cv2.contourArea(ordered.astype(np.float32))
    if quad_area <= 0:
        return None

    rect = cv2.minAreaRect(contour)
    rect_w, rect_h = rect[1]
    rect_area = max(rect_w * rect_h, 1.0)
    fill_ratio = float(quad_area / rect_area)
    area_ratio = quad_area / max(frame_area, 1.0)
    ratio = _quad_aspect_ratio(ordered)
    ratio_error = _ratio_error(ratio, config.target_aspect_ratio)
    right_angle_error = _right_angle_error(ordered)
    edge_strength = _contour_edge_strength(edge_binary, contour)
    score = _rectangle_score(area_ratio, fill_ratio, ratio_error, right_angle_error)
    return _RectangleCandidate(
        quad=ordered,
        area_ratio=area_ratio,
        fill_ratio=fill_ratio,
        edge_strength=edge_strength,
        ratio=ratio,
        ratio_error=ratio_error,
        right_angle_error=right_angle_error,
        score=score,
    )


def _build_roi_debug(
    config: PreprocessConfig,
    candidate_count: int,
    best_candidate: _RectangleCandidate | None,
    roi_found: bool,
) -> ROIDebugInfo:
    if best_candidate is None:
        return ROIDebugInfo(
            roi_found=False,
            failure_reason="no contour candidate",
            best_candidate_source="none",
            best_candidate_type="none",
            best_candidate_area_ratio=None,
            best_candidate_component_area_ratio=None,
            best_candidate_rect_fill_ratio=None,
            best_candidate_edge_strength=None,
            best_candidate_ratio=None,
            best_candidate_ratio_error=None,
            component_count=0,
            component_rank=None,
            corner_found=False,
            min_area_ratio_threshold=config.min_roi_area_ratio,
            edge_threshold=float(config.edge_threshold),
            rectangle_ratio_tolerance=config.rectangle_ratio_tolerance,
            quadrilateral_ratio_tolerance=config.quadrilateral_ratio_tolerance,
            white_threshold=float(config.white_threshold),
            candidate_count=candidate_count,
        )

    failure_reason: str | None = None
    if not roi_found:
        if best_candidate.area_ratio < config.min_roi_area_ratio:
            failure_reason = "area below threshold"
        elif best_candidate.edge_strength < float(config.edge_threshold):
            failure_reason = "edge too weak"
        elif best_candidate.fill_ratio < 0.70:
            failure_reason = "rectangle fill too low"
        elif best_candidate.ratio_error > config.rectangle_ratio_tolerance:
            failure_reason = "rectangle ratio mismatch"
        elif best_candidate.right_angle_error > 0.35:
            failure_reason = "corners not rectangular"
        else:
            failure_reason = "no valid roi after filtering"

    return ROIDebugInfo(
        roi_found=roi_found,
        failure_reason=failure_reason,
        best_candidate_source="rect_contour",
        best_candidate_type="rectangle",
        best_candidate_area_ratio=best_candidate.area_ratio,
        best_candidate_component_area_ratio=None,
        best_candidate_rect_fill_ratio=best_candidate.fill_ratio,
        best_candidate_edge_strength=best_candidate.edge_strength,
        best_candidate_ratio=best_candidate.ratio,
        best_candidate_ratio_error=best_candidate.ratio_error,
        component_count=0,
        component_rank=1,
        corner_found=True,
        min_area_ratio_threshold=config.min_roi_area_ratio,
        edge_threshold=float(config.edge_threshold),
        rectangle_ratio_tolerance=config.rectangle_ratio_tolerance,
        quadrilateral_ratio_tolerance=config.quadrilateral_ratio_tolerance,
        white_threshold=float(config.white_threshold),
        candidate_count=candidate_count,
    )


def build_board_debug_image(board_binary: Image.Image, ordered_quad: np.ndarray | None) -> Image.Image:
    cv2 = _import_cv2()
    debug_bgr = cv2.cvtColor(np.asarray(board_binary.convert("L")), cv2.COLOR_GRAY2BGR)
    if ordered_quad is not None:
        points = np.array(ordered_quad, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(debug_bgr, [points], isClosed=True, color=(0, 255, 255), thickness=3)
    return Image.fromarray(cv2.cvtColor(debug_bgr, cv2.COLOR_BGR2RGB))


def detect_board(image: Image.Image, config: PreprocessConfig) -> BoardDetectionResult:
    cv2 = _import_cv2()
    gray = np.asarray(image.convert("L"))
    edge_binary = _build_board_binary(gray, config)
    board_binary = Image.fromarray(edge_binary, mode="L")
    contours, _ = cv2.findContours(edge_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(gray.shape[0] * gray.shape[1])

    best_any: _RectangleCandidate | None = None
    best_valid: _RectangleCandidate | None = None

    for contour in contours:
        candidate = _extract_rectangle_candidate(contour, edge_binary, frame_area, config)
        if candidate is None:
            continue
        if best_any is None or candidate.score > best_any.score:
            best_any = candidate

        if candidate.area_ratio < config.min_roi_area_ratio:
            continue
        if candidate.edge_strength < float(config.edge_threshold):
            continue
        if candidate.fill_ratio < 0.70:
            continue
        if candidate.ratio_error > config.rectangle_ratio_tolerance:
            continue
        if candidate.right_angle_error > 0.35:
            continue
        if best_valid is None or candidate.score > best_valid.score:
            best_valid = candidate

    chosen = best_valid or best_any
    if chosen is None:
        roi_debug = _build_roi_debug(config=config, candidate_count=0, best_candidate=None, roi_found=False)
        return BoardDetectionResult(
            roi_found=False,
            roi_quad=None,
            ordered_quad=None,
            board_binary=board_binary,
            debug_image=build_board_debug_image(board_binary, None),
            roi_debug=roi_debug,
            candidate_source=roi_debug.best_candidate_source,
            candidate_type=roi_debug.best_candidate_type,
            candidate_count=roi_debug.candidate_count,
            component_count=0,
        )

    roi_found = best_valid is not None
    ordered_quad = None
    if roi_found:
        expanded = expand_quad(chosen.quad, config.roi_padding, image.width, image.height)
        ordered_quad = order_quad(expanded)

    roi_debug = _build_roi_debug(
        config=config,
        candidate_count=len(contours),
        best_candidate=chosen,
        roi_found=roi_found,
    )
    return BoardDetectionResult(
        roi_found=roi_found,
        roi_quad=None if ordered_quad is None else tuple((int(point[0]), int(point[1])) for point in ordered_quad),
        ordered_quad=ordered_quad,
        board_binary=board_binary,
        debug_image=build_board_debug_image(board_binary, ordered_quad),
        roi_debug=roi_debug,
        candidate_source=roi_debug.best_candidate_source,
        candidate_type=roi_debug.best_candidate_type,
        candidate_count=roi_debug.candidate_count,
        component_count=0,
    )
