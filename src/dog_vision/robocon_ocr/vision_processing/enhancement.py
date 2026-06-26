from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

from robocon_ocr.config import PreprocessConfig


@dataclass(slots=True)
class EnhancementResult:
    denoised: Image.Image
    binary: Image.Image
    prepared_for_ocr: Image.Image


def _import_cv2():
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV。请先执行 `pip install -r requirements.txt`。") from exc
    return cv2


def _apply_local_contrast(gray: np.ndarray, config: PreprocessConfig) -> np.ndarray:
    clip_limit = max(0.0, float(config.enhance_contrast_clip_limit))
    tile_grid_size = max(1, int(config.enhance_contrast_tile_grid_size))
    if clip_limit <= 0.0:
        return gray
    cv2 = _import_cv2()
    clahe = cv2.createCLAHE(
        clipLimit=clip_limit,
        tileGridSize=(tile_grid_size, tile_grid_size),
    )
    return clahe.apply(gray)


def _remove_small_foreground_components(binary: np.ndarray, min_area: int) -> np.ndarray:
    area_threshold = max(0, int(min_area))
    if area_threshold <= 0:
        return binary
    cv2 = _import_cv2()
    foreground = (binary == 0).astype(np.uint8) * 255
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(foreground, connectivity=8)
    cleaned_foreground = np.zeros_like(foreground)
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area >= area_threshold:
            cleaned_foreground[labels == label] = 255
    return np.where(cleaned_foreground > 0, 0, 255).astype(np.uint8)


def enhance_for_ocr(image: Image.Image, config: PreprocessConfig) -> EnhancementResult:
    cv2 = _import_cv2()
    gray = image.convert("L")
    if config.scale_factor != 1.0:
        gray = gray.resize(
            (
                max(1, int(gray.width * config.scale_factor)),
                max(1, int(gray.height * config.scale_factor)),
            ),
            Image.Resampling.LANCZOS,
        )
    arr = np.asarray(gray, dtype=np.uint8)
    contrast_arr = _apply_local_contrast(arr, config)
    _, binary = cv2.threshold(contrast_arr, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    binary = _remove_small_foreground_components(binary, config.enhance_remove_noise_area_min)
    binary_image = Image.fromarray(binary.astype(np.uint8), mode="L")
    return EnhancementResult(
        denoised=Image.fromarray(contrast_arr, mode="L"),
        binary=binary_image,
        prepared_for_ocr=binary_image,
    )
