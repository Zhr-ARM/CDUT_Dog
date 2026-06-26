from __future__ import annotations

import json
from collections.abc import Iterable

import numpy as np
from PIL import Image

from robocon_ocr.config import OCRConfig
from robocon_ocr.image_recognition.base import OCRResult
from robocon_ocr.result.expression import normalize_ocr_text, validate_ocr_text


class LightweightMathRecognizer:
    backend_name = "lightweight"
    supports_fallback_variants = False

    def __init__(self, config: OCRConfig) -> None:
        self.config = config
        self._engine = None

    def _resolve_device(self) -> str | None:
        requested = self.config.device.strip().lower()
        if requested == "auto":
            return None
        if requested in {"gpu", "cuda"}:
            return "gpu:0"
        return requested

    def _build_engine(self):
        try:
            from paddleocr import TextRecognition
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "未安装 PaddleOCR 轻量识别依赖。"
                "请参考 PaddleOCR 官方文档安装 `paddleocr` 与对应 CUDA/CPU 版本的 `paddlepaddle`。"
            ) from exc

        kwargs = self._build_engine_kwargs()
        device = self._resolve_device()
        if device is not None:
            kwargs["device"] = device
        try:
            return TextRecognition(**kwargs)
        except RuntimeError as exc:
            # Some PaddleOCR installs expose the HPI engine path but require
            # extra runtime packages such as `ultra-infer`. Fall back to the
            # standard predictor automatically so camera mode can start cleanly.
            if kwargs.get("enable_hpi") and "ultra-infer" in str(exc):
                fallback_kwargs = dict(kwargs)
                fallback_kwargs["enable_hpi"] = False
                return TextRecognition(**fallback_kwargs)
            raise

    def _build_engine_kwargs(self) -> dict[str, object]:
        return {
            "model_name": self.config.lightweight_model_name,
            "enable_hpi": False,
        }

    @property
    def engine(self):
        if self._engine is None:
            self._engine = self._build_engine()
        return self._engine

    def warmup(self) -> None:
        _ = self.engine

    def recognize(self, image: Image.Image) -> OCRResult:
        image_array = np.asarray(image.convert("RGB"))
        try:
            predictions = self.engine.predict(input=image_array, batch_size=1)
        except Exception as exc:
            raise RuntimeError(f"PaddleOCR 轻量推理失败: {exc}") from exc

        raw_text, confidence = self._extract_prediction(predictions)
        raw_text = raw_text.strip()
        if not raw_text:
            return OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                psm=None,
                error="no text detected by OCR",
                backend=self.backend_name,
            )

        normalized = normalize_ocr_text(raw_text)
        error = validate_ocr_text(normalized) if self.config.strict_charset else None
        if error is not None:
            return OCRResult(
                raw_text=raw_text,
                confidence=0.0,
                lines=[],
                psm=None,
                error=error,
                backend=self.backend_name,
            )

        return OCRResult(
            raw_text=normalized,
            confidence=confidence,
            lines=[normalized] if normalized else [],
            psm=None,
            error=None if normalized else "no text detected by OCR",
            backend=self.backend_name,
        )

    def _extract_prediction(self, predictions) -> tuple[str, float]:
        items = list(predictions) if isinstance(predictions, Iterable) and not isinstance(predictions, (str, bytes)) else [predictions]
        if not items:
            return "", 0.0
        payload = self._coerce_payload(items[0])
        text = str(payload.get("rec_text") or "").strip()
        confidence = float(payload.get("rec_score") or 0.0)
        return text, confidence

    def _coerce_payload(self, value) -> dict[str, object]:
        if isinstance(value, dict):
            payload = value.get("res", value)
            if isinstance(payload, list):
                return self._coerce_payload(payload[0] if payload else {})
            return payload if isinstance(payload, dict) else {}

        json_payload = getattr(value, "json", None)
        if json_payload is not None:
            if callable(json_payload):
                json_payload = json_payload()
            if isinstance(json_payload, str):
                try:
                    return self._coerce_payload(json.loads(json_payload))
                except json.JSONDecodeError:
                    return {}
            return self._coerce_payload(json_payload)

        res_payload = getattr(value, "res", None)
        if res_payload is not None:
            return self._coerce_payload(res_payload)
        return {}
