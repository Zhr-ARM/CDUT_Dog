from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

from PIL import Image

from robocon_ocr.config import OCRConfig, PipelineConfig, PreprocessConfig
from robocon_ocr.image_recognition.base import MathTextRecognizer, OCRResult
from robocon_ocr.image_recognition.factory import create_recognizer
from robocon_ocr.result.expression import ParsedExpression, parse_expression
from robocon_ocr.result.reporter import PipelineRecord
from robocon_ocr.vision_processing.board_detection import BoardDetectionResult
from robocon_ocr.vision_processing.board_detection import detect_board
from robocon_ocr.vision_processing.enhancement import EnhancementResult, enhance_for_ocr
from robocon_ocr.vision_processing.expression_region import ExpressionRegionResult
from robocon_ocr.vision_processing.expression_region import extract_expression_region
from robocon_ocr.vision_processing.rectification import RectificationResult, rectify_board

StageName = Literal["board_detection", "rectification", "expression_region", "enhancement", "segmentation", "ocr", "postprocess"]
STAGE_SEQUENCE: tuple[StageName, ...] = (
    "board_detection",
    "rectification",
    "expression_region",
    "enhancement",
    "segmentation",
    "ocr",
    "postprocess",
)


@dataclass(slots=True)
class SegmentationResult:
    segments: list[Image.Image] = field(default_factory=list)
    debug_image: Image.Image | None = None
    line_boxes: list[tuple[int, int, int, int]] = field(default_factory=list)


@dataclass(slots=True)
class OCRStageResult:
    candidates: list[OCRResult] = field(default_factory=list)
    best: OCRResult | None = None
    error: str | None = None


@dataclass(slots=True)
class PipelineDebugSnapshot:
    stage_name: StageName
    image: Image.Image | None
    lines: list[str] = field(default_factory=list)


@dataclass(slots=True)
class PipelineContext:
    image_name: str
    original: Image.Image
    preprocess_config: PreprocessConfig
    ocr_config: OCRConfig
    board_detection: BoardDetectionResult | None = None
    rectification: RectificationResult | None = None
    expression_region: ExpressionRegionResult | None = None
    enhancement: EnhancementResult | None = None
    segmentation: SegmentationResult | None = None
    ocr_stage: OCRStageResult | None = None
    parsed: ParsedExpression | None = None
    label: object | None = None
    stop_after_stage: StageName | None = None
    snapshots: dict[str, PipelineDebugSnapshot] = field(default_factory=dict)

    @property
    def completed_stage(self) -> StageName | None:
        if self.parsed is not None:
            return "postprocess"
        if self.ocr_stage is not None:
            return "ocr"
        if self.segmentation is not None:
            return "segmentation"
        if self.enhancement is not None:
            return "enhancement"
        if self.expression_region is not None:
            return "expression_region"
        if self.rectification is not None:
            return "rectification"
        if self.board_detection is not None:
            return "board_detection"
        return None




def _snapshot(stage_name: StageName, image: Image.Image | None, *lines: str) -> PipelineDebugSnapshot:
    return PipelineDebugSnapshot(stage_name=stage_name, image=image, lines=[line for line in lines if line])


def _build_stage_lines(context: PipelineContext, stage_name: StageName) -> list[str]:
    lines = [f"Stage: {stage_name}"]
    board = context.board_detection
    if board is not None:
        lines.append(f"ROI: {board.roi_found}")
        if board.roi_debug.failure_reason:
            lines.append(f"Reason: {board.roi_debug.failure_reason}")
    if context.ocr_stage and context.ocr_stage.best is not None:
        lines.append(f"OCR: {context.ocr_stage.best.raw_text or '<empty>'}")
    if context.parsed is not None:
        lines.append(f"Expr: {context.parsed.expression or '<empty>'}")
        lines.append(f"Valid: {context.parsed.is_valid}")
        if context.parsed.answer is not None:
            lines.append(f"Answer: {context.parsed.answer}")
        if context.parsed.error:
            lines.append(f"Error: {context.parsed.error}")
    return lines


class _BoardDetectionStage:
    name: StageName = "board_detection"

    def run(self, context: PipelineContext) -> PipelineContext:
        result = detect_board(context.original, context.preprocess_config)
        context.board_detection = result
        context.snapshots[self.name] = _snapshot(self.name, result.debug_image, *self._lines(result))
        return context

    def _lines(self, result: BoardDetectionResult) -> list[str]:
        return [
            f"ROI: {result.roi_found}",
            f"Source: {result.candidate_source}",
            f"Type: {result.candidate_type}",
            f"Candidates: {result.candidate_count}",
            f"Components: {result.component_count}",
            f"Reason: {result.roi_debug.failure_reason or 'passed'}",
        ]


class _RectificationStage:
    name: StageName = "rectification"

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.board_detection is None or not context.board_detection.roi_found or context.board_detection.ordered_quad is None:
            return context
        result = rectify_board(context.original, context.board_detection.ordered_quad, context.preprocess_config)
        context.rectification = result
        context.snapshots[self.name] = _snapshot(
            self.name,
            result.rectified,
            "Perspective: ready",
            f"Crop: {result.cropped.width}x{result.cropped.height}",
            f"Rectified: {result.rectified.width}x{result.rectified.height}",
        )
        return context


class _ExpressionRegionStage:
    name: StageName = "expression_region"

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.rectification is None:
            return context
        result = extract_expression_region(context.rectification.rectified, context.preprocess_config)
        context.expression_region = result
        context.snapshots[self.name] = _snapshot(
            self.name,
            result.debug_image,
            f"Region: {result.region_found}",
            f"BBox: {result.bbox}",
            f"Rows: {result.row_range}",
            f"Cols: {result.col_range}",
            f"Otsu: {result.otsu_threshold}",
            f"RowPeak: {result.projection_summary.row_peak:.4f}",
            f"RowThr: {result.projection_summary.row_enter_threshold:.4f}/{result.projection_summary.row_exit_threshold:.4f}",
            f"ColPeak: {result.projection_summary.col_peak:.4f}",
            f"ColThr: {result.projection_summary.col_enter_threshold:.4f}/{result.projection_summary.col_exit_threshold:.4f}",
            f"Search: {result.search_window}",
            f"Effective: {result.effective_search_window}",
            f"Reason: {result.failure_reason or 'passed'}",
        )
        return context


class _EnhancementStage:
    name: StageName = "enhancement"

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.expression_region is None or not context.expression_region.region_found or context.expression_region.cropped_region is None:
            return context
        result = enhance_for_ocr(context.expression_region.cropped_region, context.preprocess_config)
        context.enhancement = result
        context.snapshots[self.name] = _snapshot(
            self.name,
            result.prepared_for_ocr,
            "Enhancement: ready",
            f"Prepared: {result.prepared_for_ocr.width}x{result.prepared_for_ocr.height}",
        )
        return context


class _SegmentationStage:
    name: StageName = "segmentation"

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.enhancement is None:
            return context
        result = SegmentationResult(
            segments=[context.enhancement.prepared_for_ocr],
            debug_image=context.enhancement.prepared_for_ocr,
            line_boxes=[(0, 0, context.enhancement.prepared_for_ocr.width, context.enhancement.prepared_for_ocr.height)],
        )
        context.segmentation = result
        context.snapshots[self.name] = _snapshot(
            self.name,
            result.debug_image,
            "Segmentation: placeholder",
            f"Segments: {len(result.segments)}",
        )
        return context


class _OCRStage:
    name: StageName = "ocr"

    def __init__(self, recognizer: MathTextRecognizer | None = None) -> None:
        self._recognizer = recognizer

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.enhancement is None:
            return context
        recognizer = self._recognizer or create_recognizer(context.ocr_config)

        # 联网大模型传彩色纠正图，本地模型传二值化增强图
        if recognizer.backend_name == "api" and context.rectification is not None:
            ocr_input = context.rectification.rectified.convert("RGB")
            preview = context.rectification.rectified
        else:
            ocr_input = context.enhancement.prepared_for_ocr.convert("RGB")
            preview = context.enhancement.prepared_for_ocr

        # 非阻塞路径（联网 API）— poll 消费结果，submit 受冷却控制，不阻塞流水线
        api_status = ""
        if getattr(recognizer, "is_async", False):
            result = recognizer.poll()
            if result is not None:
                best = result
                api_status = "result ready"
            else:
                recognizer.submit(ocr_input)
                api_status = recognizer.status
                cached = getattr(recognizer, "_cached_result", None)
                if cached is not None:
                    best = cached
                else:
                    best = OCRResult(raw_text="", confidence=0.0, lines=[], error="pending", backend="api")
        else:
            best = recognizer.recognize(ocr_input)

        stage_result = OCRStageResult(
            candidates=[best],
            best=best,
            error=best.error,
        )
        context.ocr_stage = stage_result
        context.snapshots[self.name] = _snapshot(
            self.name,
            preview,
            "OCR complete",
            f"Backend: {best.backend}",
            f"Best: {best.raw_text if best else '<empty>'}",
            f"Error: {stage_result.error or 'none'}",
            f"API: {api_status}" if api_status else "",
        )
        return context


class _PostprocessStage:
    name: StageName = "postprocess"

    def run(self, context: PipelineContext) -> PipelineContext:
        if context.ocr_stage is None or context.ocr_stage.best is None:
            if context.board_detection is None or not context.board_detection.roi_found:
                context.parsed = ParsedExpression("", "", None, False, "roi not found")
            elif context.expression_region is not None and not context.expression_region.region_found:
                context.parsed = ParsedExpression(
                    "",
                    "",
                    None,
                    False,
                    context.expression_region.failure_reason or "expression region not found",
                )
            else:
                context.parsed = ParsedExpression("", "", None, False, "ocr not run")
        else:
            context.parsed = parse_expression(context.ocr_stage.best.raw_text)
        image = None
        if context.enhancement is not None:
            image = context.enhancement.prepared_for_ocr
        elif context.expression_region is not None:
            image = context.expression_region.debug_image
        elif context.rectification is not None:
            image = context.rectification.rectified
        context.snapshots[self.name] = _snapshot(self.name, image, *_build_stage_lines(context, self.name))
        return context


def _should_stop(stage_name: StageName, stop_after: StageName | None) -> bool:
    return stop_after == stage_name


def run_staged_pipeline(
    context: PipelineContext,
    stop_after: StageName | None = None,
    recognizer: MathTextRecognizer | None = None,
) -> PipelineContext:
    stages = (
        _BoardDetectionStage(),
        _RectificationStage(),
        _ExpressionRegionStage(),
        _EnhancementStage(),
        _SegmentationStage(),
        _OCRStage(recognizer=recognizer),
        _PostprocessStage(),
    )
    effective_stop = stop_after or context.stop_after_stage
    for stage in stages:
        context = stage.run(context)
        if _should_stop(stage.name, effective_stop):
            break
    return context


def run_dataset_pipeline_image(
    image: Image.Image,
    image_name: str,
    config: PipelineConfig,
    recognizer: MathTextRecognizer | None = None,
) -> PipelineContext:
    context = PipelineContext(
        image_name=image_name,
        original=image.convert("RGB"),
        preprocess_config=config.preprocess,
        ocr_config=config.ocr,
        stop_after_stage=config.stop_after_stage,
    )
    return run_staged_pipeline(context, stop_after=config.stop_after_stage, recognizer=recognizer)


def run_camera_pipeline_frame(
    frame: Image.Image,
    image_name: str,
    config: PipelineConfig,
    recognizer: MathTextRecognizer | None = None,
) -> PipelineContext:
    context = PipelineContext(
        image_name=image_name,
        original=frame.convert("RGB"),
        preprocess_config=config.preprocess,
        ocr_config=config.ocr,
        stop_after_stage=config.stop_after_stage,
    )
    return run_staged_pipeline(context, stop_after=config.stop_after_stage, recognizer=recognizer)


def context_to_record(context: PipelineContext) -> PipelineRecord:
    board = context.board_detection
    roi_found = bool(board and board.roi_found)
    roi_quad = None if board is None else board.roi_quad
    ocr_result = OCRResult(
        raw_text="",
        confidence=0.0,
        lines=[],
        psm=None,
        error="roi not found",
        backend=context.ocr_config.backend,
    )
    if board is not None and board.roi_found:
        if context.ocr_stage and context.ocr_stage.best is not None:
            ocr_result = context.ocr_stage.best
        elif context.stop_after_stage is not None and STAGE_SEQUENCE.index(context.stop_after_stage) < STAGE_SEQUENCE.index("ocr"):
            ocr_result = OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                psm=None,
                error="ocr not run",
                backend=context.ocr_config.backend,
            )
        else:
            ocr_result = OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                psm=None,
                error="no text detected by OCR",
                backend=context.ocr_config.backend,
            )
    parsed = context.parsed
    if parsed is None:
        if not roi_found:
            parsed = ParsedExpression("", "", None, False, "roi not found")
        elif context.ocr_stage is not None and context.ocr_stage.best is not None:
            parsed = parse_expression(context.ocr_stage.best.raw_text)
        else:
            parsed = ParsedExpression("", "", None, False, "pipeline stopped before postprocess")
    return PipelineRecord(
        image_name=context.image_name,
        ocr=ocr_result,
        parsed=parsed,
        label=context.label,
        roi_found=roi_found,
        roi_quad=roi_quad,
    )


def save_stage_debug_images(context: PipelineContext, debug_dir: Path) -> None:
    debug_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(context.image_name).stem
    for stage_name in STAGE_SEQUENCE:
        snapshot = context.snapshots.get(stage_name)
        if snapshot is None or snapshot.image is None:
            continue
        snapshot.image.save(debug_dir / f"{stem}_{stage_name}.png")
    if context.expression_region is not None and context.expression_region.mask_debug_image is not None:
        context.expression_region.mask_debug_image.save(debug_dir / f"{stem}_expression_region_mask.png")
