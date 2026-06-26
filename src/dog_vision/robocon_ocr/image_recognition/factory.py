from __future__ import annotations

from robocon_ocr.config import OCRConfig
from robocon_ocr.image_recognition.onnx_recognizer import OnnxMathRecognizer


def create_recognizer(config: OCRConfig):
    backend = config.backend.strip().lower()
    if backend == "onnx":
        return OnnxMathRecognizer(config)
    if backend == "api":
        from robocon_ocr.image_recognition.api_recognizer import APIMathRecognizer

        return APIMathRecognizer(config)
    raise ValueError(f"unsupported OCR backend: {config.backend}")
