from __future__ import annotations

from collections.abc import Callable

from robocon_ocr.recognition_output import RecognitionOutput


class RecognitionFilter:
    """Consensus filter for recognition results.

    - **api** backend: emits every valid result immediately (already highly accurate).
    - **onnx** backend: requires *consensus* consecutive frames
      where ``(expression, answer)`` are identical before emitting.
    """

    def __init__(
        self,
        backend: str,
        consensus: int = 3,
        on_emit: Callable[[RecognitionOutput], None] | None = None,
    ) -> None:
        self._backend = backend
        self._consensus = 1 if backend == "api" else max(1, consensus)
        self._on_emit = on_emit
        self._buffer: list[RecognitionOutput] = []

    @property
    def consensus(self) -> int:
        return self._consensus

    def feed(self, output: RecognitionOutput) -> RecognitionOutput | None:
        """Feed a new recognition result.

        Returns the emitted ``RecognitionOutput`` when the consensus threshold
        is reached, or ``None`` when still accumulating.
        """
        if self._consensus == 1:
            # API backend — pass through every valid, non-duplicate result
            if output.is_valid:
                if self._on_emit:
                    self._on_emit(output)
                return output
            return None

        # Local model consensus path
        if not output.is_valid:
            self._buffer.clear()
            return None

        if self._buffer:
            last = self._buffer[-1]
            if output.expression == last.expression and output.answer == last.answer:
                self._buffer.append(output)
            else:
                self._buffer = [output]
        else:
            self._buffer = [output]

        if len(self._buffer) >= self._consensus:
            emitted = self._buffer[-1]
            self._buffer.clear()
            if self._on_emit:
                self._on_emit(emitted)
            return emitted

        return None

    def reset(self) -> None:
        """Clear the internal buffer, restarting the consensus count."""
        self._buffer.clear()
