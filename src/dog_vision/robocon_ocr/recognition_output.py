from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class RecognitionOutput:
    """Standardized recognition result for parent-project integration.

    When ``is_valid`` is True the expression was successfully parsed and
    ``answer`` / ``answer_mod_4`` are guaranteed to be non-None.
    """

    expression: str
    answer: int | None
    answer_mod_4: int | None
    is_valid: bool
    confidence: float
    backend: str
    error: str | None = None
