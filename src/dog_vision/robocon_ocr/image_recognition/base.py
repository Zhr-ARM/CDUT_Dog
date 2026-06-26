from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from PIL import Image


@dataclass(slots=True)
class OCRResult:
    raw_text: str
    confidence: float
    lines: list[str]
    psm: int | None = None
    error: str | None = None
    backend: str = "unknown"


class MathTextRecognizer(Protocol):
    backend_name: str
    supports_fallback_variants: bool

    def warmup(self) -> None: ...

    def recognize(self, image: Image.Image) -> OCRResult: ...
