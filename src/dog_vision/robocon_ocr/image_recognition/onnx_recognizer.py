from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

from robocon_ocr.config import OCRConfig
from robocon_ocr.image_recognition.base import OCRResult
from robocon_ocr.result.expression import normalize_ocr_text, validate_ocr_text


class OnnxMathRecognizer:
    backend_name = "onnx"
    supports_fallback_variants = False

    def __init__(self, config: OCRConfig) -> None:
        self.config = config
        self._session = None
        self._char_dict: list[str] | None = None

    def _load_resources(self) -> None:
        import onnxruntime as ort

        model_path = self._resolve_resource(self.config.onnx_model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        dict_path = self._resolve_resource(self.config.onnx_dict_path)
        if not dict_path.is_file():
            raise FileNotFoundError(f"Character dictionary not found: {dict_path}")

        self._session = ort.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"],
        )
        with open(dict_path, "r", encoding="utf-8") as f:
            self._char_dict = [line.rstrip("\n") for line in f]

    def _resolve_resource(self, configured_path: str) -> Path:
        path = Path(configured_path).expanduser()
        if path.is_file() or path.is_absolute():
            return path

        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parents[2] / path,
        ]
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(Path(get_package_share_directory("dog_vision")) / path)
        except Exception:
            pass

        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return path

    @property
    def session(self):
        if self._session is None:
            self._load_resources()
        return self._session

    @property
    def char_dict(self) -> list[str]:
        if self._char_dict is None:
            self._load_resources()
        # _load_resources guarantees _char_dict is set
        assert self._char_dict is not None
        return self._char_dict

    def warmup(self) -> None:
        _ = self.session, self.char_dict

    def recognize(self, image: Image.Image) -> OCRResult:
        import cv2

        arr = np.array(image.convert("RGB"))[:, :, ::-1].copy().astype(np.float32)
        h, w = arr.shape[:2]
        ratio = 48.0 / h
        new_w = int(w * ratio)
        arr = cv2.resize(arr, (new_w, 48), interpolation=cv2.INTER_LINEAR)
        arr = (arr - 127.5) / 127.5
        arr = np.transpose(arr, (2, 0, 1))
        arr = np.expand_dims(arr, axis=0)

        outputs = self.session.run(None, {"x": arr})
        logits = outputs[0]  # [1, T, 18385]

        raw_text, confidence = self._ctc_greedy_decode(logits)
        raw_text = raw_text.strip()
        if not raw_text:
            return OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
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
                error=error,
                backend=self.backend_name,
            )

        return OCRResult(
            raw_text=normalized,
            confidence=confidence,
            lines=[normalized] if normalized else [],
            error=None if normalized else "no text detected by OCR",
            backend=self.backend_name,
        )

    def _ctc_greedy_decode(self, logits: np.ndarray) -> tuple[str, float]:
        """CTC greedy decode.

        Class 0 = blank, class i (1-based) = char_dict[i-1].
        """
        pred_ids = np.argmax(logits[0], axis=-1)  # [T]
        confidences = np.max(logits[0], axis=-1)
        chars: list[str] = []
        confs: list[float] = []
        prev = -1
        for class_id, conf in zip(pred_ids, confidences):
            cid = int(class_id)
            if cid != prev and cid != 0:
                dict_idx = cid - 1
                if 0 <= dict_idx < len(self.char_dict):
                    chars.append(self.char_dict[dict_idx])
                    confs.append(float(conf))
            prev = cid
        text = "".join(chars)
        confidence = float(np.mean(confs)) if confs else 0.0
        return text, confidence
