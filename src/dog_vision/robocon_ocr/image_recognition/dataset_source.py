from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class LabelRecord:
    filename: str
    expression: str
    answer: int
    font_size_px: int


def list_images(dataset_dir: Path) -> list[Path]:
    return sorted(
        path
        for path in dataset_dir.iterdir()
        if path.is_file() and path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}
    )


def load_labels(label_file: Path) -> dict[str, LabelRecord]:
    records: dict[str, LabelRecord] = {}
    with label_file.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle, delimiter="\t")
        for row in reader:
            records[row["filename"]] = LabelRecord(
                filename=row["filename"],
                expression=row["expression"].strip(),
                answer=int(row["answer"]),
                font_size_px=int(row["font_size_px"]),
            )
    return records

