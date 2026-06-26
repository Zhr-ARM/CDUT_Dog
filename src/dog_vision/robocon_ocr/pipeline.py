from __future__ import annotations

from PIL import Image

from robocon_ocr.config import PipelineConfig
from robocon_ocr.image_recognition.dataset_source import list_images, load_labels
from robocon_ocr.image_recognition.factory import create_recognizer
from robocon_ocr.image_recognition.preprocess import (
    save_debug_images,
    prepare_image_for_ocr,
)
from robocon_ocr.result.reporter import PipelineRecord
from robocon_ocr.staged_pipeline import context_to_record
from robocon_ocr.staged_pipeline import run_dataset_pipeline_image
from robocon_ocr.staged_pipeline import save_stage_debug_images


def _save_legacy_debug_outputs(
    image_name: str,
    image: Image.Image,
    config: PipelineConfig,
) -> None:
    if config.debug_dir is None:
        return
    preprocess_result = prepare_image_for_ocr(image.convert("RGB"), config.preprocess)
    save_debug_images(
        image_name,
        preprocess_result.cropped,
        preprocess_result.prepared,
        config.debug_dir,
        rectified=preprocess_result.rectified,
    )


def run_pipeline(config: PipelineConfig) -> list[PipelineRecord]:
    image_paths = list_images(config.dataset_dir)
    labels = load_labels(config.label_file) if config.label_file else {}
    recognizer = create_recognizer(config.ocr)
    records: list[PipelineRecord] = []
    warmed_up = False

    for image_path in image_paths:
        image = Image.open(image_path).convert("RGB")
        context = run_dataset_pipeline_image(
            image=image,
            image_name=image_path.name,
            config=config,
            recognizer=recognizer,
        )
        context.label = labels.get(image_path.name)
        if context.board_detection is not None and context.board_detection.roi_found and config.ocr.warmup and not warmed_up:
            recognizer.warmup()
            warmed_up = True
        if config.debug_dir is not None:
            _save_legacy_debug_outputs(image_path.name, image, config)
            if config.debug_save_stages:
                save_stage_debug_images(context, config.debug_dir)
        records.append(context_to_record(context))

    return records


def run_image_pipeline(
    image: Image.Image,
    image_name: str,
    config: PipelineConfig,
) -> PipelineRecord:
    recognizer = create_recognizer(config.ocr)
    if config.ocr.warmup:
        recognizer.warmup()
    context = run_dataset_pipeline_image(
        image=image,
        image_name=image_name,
        config=config,
        recognizer=recognizer,
    )
    if config.debug_dir is not None:
        _save_legacy_debug_outputs(image_name, image, config)
        if config.debug_save_stages:
            save_stage_debug_images(context, config.debug_dir)
    return context_to_record(context)
