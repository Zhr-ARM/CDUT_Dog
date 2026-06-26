from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

from robocon_ocr.config import PreprocessConfig


@dataclass(slots=True)
class RectificationResult:
    cropped: Image.Image
    rectified: Image.Image
    transform_matrix: np.ndarray


def _import_cv2():
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("未安装 OpenCV。请先执行 `pip install -r requirements.txt`。") from exc
    return cv2


def bounding_crop(image: Image.Image, quad: np.ndarray) -> Image.Image:
    xs = quad[:, 0]
    ys = quad[:, 1]
    x0 = max(0, int(np.floor(xs.min())))
    y0 = max(0, int(np.floor(ys.min())))
    x1 = min(image.width, int(np.ceil(xs.max())) + 1)
    y1 = min(image.height, int(np.ceil(ys.max())) + 1)
    return image.crop((x0, y0, x1, y1))


def rectify_board(image: Image.Image, quad: np.ndarray, config: PreprocessConfig) -> RectificationResult:
    cv2 = _import_cv2()
    rgb = np.asarray(image.convert("RGB"))
    destination = np.array(
        [
            [0, 0],
            [config.perspective_width - 1, 0],
            [config.perspective_width - 1, config.perspective_height - 1],
            [0, config.perspective_height - 1],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(quad.astype(np.float32), destination)
    warped = cv2.warpPerspective(
        rgb,
        matrix,
        (config.perspective_width, config.perspective_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    cropped = bounding_crop(image.convert("RGB"), quad)
    return RectificationResult(
        cropped=cropped,
        rectified=Image.fromarray(warped),
        transform_matrix=matrix,
    )
