from __future__ import annotations

from dataclasses import dataclass

from robocon_ocr.image_recognition.dataset_source import LabelRecord
from robocon_ocr.image_recognition.base import OCRResult
from robocon_ocr.result.expression import ParsedExpression, to_expression_only


@dataclass(slots=True)
class PipelineRecord:
    image_name: str
    ocr: OCRResult
    parsed: ParsedExpression
    label: LabelRecord | None
    roi_found: bool = False
    roi_quad: tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]] | None = None

    @property
    def expression_match(self) -> bool | None:
        if self.label is None:
            return None
        return self.parsed.expression == to_expression_only(self.label.expression)

    @property
    def answer_match(self) -> bool | None:
        if self.label is None or self.parsed.answer is None:
            return None
        return self.parsed.answer == self.label.answer


def summarize(records: list[PipelineRecord]) -> dict[str, float | int]:
    total = len(records)
    with_label = [record for record in records if record.label is not None]
    expression_correct = sum(1 for record in with_label if record.expression_match)
    answer_correct = sum(1 for record in with_label if record.answer_match)
    invalid = sum(1 for record in records if not record.parsed.is_valid)
    return {
        "total": total,
        "with_label": len(with_label),
        "expression_correct": expression_correct,
        "answer_correct": answer_correct,
        "invalid": invalid,
        "expression_accuracy": expression_correct / len(with_label) if with_label else 0.0,
        "answer_accuracy": answer_correct / len(with_label) if with_label else 0.0,
    }
