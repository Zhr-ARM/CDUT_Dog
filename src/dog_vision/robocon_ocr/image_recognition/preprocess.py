from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from robocon_ocr.config import PreprocessConfig
from robocon_ocr.vision_processing.board_detection import ROIDebugInfo
from robocon_ocr.vision_processing.board_detection import detect_board
from robocon_ocr.vision_processing.rectification import bounding_crop
from robocon_ocr.vision_processing.rectification import rectify_board
from robocon_ocr.vision_processing.enhancement import enhance_for_ocr


@dataclass(slots=True)
class PreprocessResult:
    roi_found: bool
    roi_quad: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]] | None
    cropped: Image.Image
    rectified: Image.Image
    board_binary: Image.Image
    prepared: Image.Image
    roi_debug: ROIDebugInfo


def _build_fallback_result(original: Image.Image, config: PreprocessConfig) -> PreprocessResult:
    gray = original.convert("L")
    arr = np.asarray(gray)
    threshold = int(arr.mean()) if arr.size else 0
    binary = np.where(arr > threshold, 255, 0).astype(np.uint8)
    board_binary = Image.fromarray(binary, mode="L")
    roi_debug = ROIDebugInfo(
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
        candidate_count=0,
    )
    return PreprocessResult(
        roi_found=False,
        roi_quad=None,
        cropped=original,
        rectified=original,
        board_binary=board_binary,
        prepared=board_binary,
        roi_debug=roi_debug,
    )


def detect_roi(image: Image.Image, config: PreprocessConfig) -> tuple[bool, np.ndarray | None]:
    result = detect_board(image, config)
    if result.ordered_quad is None:
        return False, None
    return True, result.ordered_quad


def crop_foreground_text(image: Image.Image, config: PreprocessConfig) -> Image.Image:
    roi_found, quad = detect_roi(image, config)
    if not roi_found or quad is None:
        return image
    return bounding_crop(image, quad)


def prepare_image_for_ocr(
    original: Image.Image,
    config: PreprocessConfig,
) -> PreprocessResult:
    rgb = original.convert("RGB")
    board_result = detect_board(rgb, config)
    roi_found = board_result.roi_found and board_result.ordered_quad is not None
    if not roi_found or board_result.ordered_quad is None:
        fallback = _build_fallback_result(rgb, config)
        fallback.board_binary = board_result.board_binary
        fallback.roi_debug = board_result.roi_debug
        return fallback

    rectification_result = rectify_board(rgb, board_result.ordered_quad, config)
    enhancement_result = enhance_for_ocr(rectification_result.rectified, config)
    return PreprocessResult(
        roi_found=True,
        roi_quad=board_result.roi_quad,
        cropped=rectification_result.cropped,
        rectified=rectification_result.rectified,
        board_binary=board_result.board_binary,
        prepared=enhancement_result.prepared_for_ocr,
        roi_debug=board_result.roi_debug,
    )


def prepare_for_ocr(image_path: Path, config: PreprocessConfig) -> PreprocessResult:
    original = Image.open(image_path).convert("RGB")
    return prepare_image_for_ocr(original, config)


def save_debug_images(
    image_name: str,
    cropped: Image.Image,
    prepared: Image.Image,
    debug_dir: Path,
    rectified: Image.Image | None = None,
) -> None:
    debug_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(image_name).stem
    cropped.save(debug_dir / f"{stem}_cropped.png")
    if rectified is not None:
        rectified.save(debug_dir / f"{stem}_rectified.png")
    prepared.save(debug_dir / f"{stem}_prepared.png")
