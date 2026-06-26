from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from robocon_ocr.roi_tuning import DEFAULT_ROI_TUNING_VALUES


@dataclass(slots=True)
class OCRConfig:
    backend: str = "api"
    device: str = "auto"
    lightweight_model_name: str = "PP-OCRv5_server_rec"
    onnx_model_path: str = "models/ocr/PP-OCRv5_server_rec.onnx"
    onnx_dict_path: str = "models/ocr/dict.txt"
    warmup: bool = True
    strict_charset: bool = True
    # --- 联网大模型 API ---
    api_provider: str = "moonshot"
    api_model: str = ""
    api_key: str = ""
    api_base_url: str = ""
    api_timeout: float = 5.0
    api_max_tokens: int = 128


@dataclass(slots=True)
class PreprocessConfig:
    white_threshold: int = int(DEFAULT_ROI_TUNING_VALUES["white_threshold"])
    component_white_threshold: int = int(DEFAULT_ROI_TUNING_VALUES["component_white_threshold"])
    component_min_area_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["component_min_area_ratio"])
    component_max_candidates: int = int(DEFAULT_ROI_TUNING_VALUES["component_max_candidates"])
    component_fill_ratio_threshold: float = float(DEFAULT_ROI_TUNING_VALUES["component_fill_ratio_threshold"])
    component_padding: int = int(DEFAULT_ROI_TUNING_VALUES["component_padding"])
    corner_search_margin: int = int(DEFAULT_ROI_TUNING_VALUES["corner_search_margin"])
    edge_threshold: int = int(DEFAULT_ROI_TUNING_VALUES["edge_threshold"])
    min_roi_area_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["min_roi_area_ratio"])
    rectangularity_min: float = 0.82
    rectangle_ratio_tolerance: float = float(DEFAULT_ROI_TUNING_VALUES["rectangle_ratio_tolerance"])
    quadrilateral_ratio_tolerance: float = float(DEFAULT_ROI_TUNING_VALUES["quadrilateral_ratio_tolerance"])
    target_aspect_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["target_aspect_ratio"])
    roi_padding: int = int(DEFAULT_ROI_TUNING_VALUES["roi_padding"])
    perspective_width: int = int(DEFAULT_ROI_TUNING_VALUES["perspective_width"])
    perspective_height: int = int(DEFAULT_ROI_TUNING_VALUES["perspective_height"])
    scale_factor: float = float(DEFAULT_ROI_TUNING_VALUES["scale_factor"])
    enhance_contrast_clip_limit: float = float(DEFAULT_ROI_TUNING_VALUES["enhance_contrast_clip_limit"])
    enhance_contrast_tile_grid_size: int = int(DEFAULT_ROI_TUNING_VALUES["enhance_contrast_tile_grid_size"])
    enhance_remove_noise_area_min: int = int(DEFAULT_ROI_TUNING_VALUES["enhance_remove_noise_area_min"])
    expression_search_top_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_search_top_ratio"])
    expression_search_bottom_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_search_bottom_ratio"])
    expression_search_left_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_search_left_ratio"])
    expression_search_right_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_search_right_ratio"])
    expression_otsu_bias: int = int(DEFAULT_ROI_TUNING_VALUES["expression_otsu_bias"])
    expression_enter_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_enter_ratio"])
    expression_exit_ratio: float = float(DEFAULT_ROI_TUNING_VALUES["expression_exit_ratio"])
    expression_min_consecutive_rows: int = int(DEFAULT_ROI_TUNING_VALUES["expression_min_consecutive_rows"])
    expression_min_consecutive_cols: int = int(DEFAULT_ROI_TUNING_VALUES["expression_min_consecutive_cols"])
    expression_bbox_padding_x: int = int(DEFAULT_ROI_TUNING_VALUES["expression_bbox_padding_x"])
    expression_bbox_padding_y: int = int(DEFAULT_ROI_TUNING_VALUES["expression_bbox_padding_y"])
    expression_bbox_expand_ratio_x: float = float(DEFAULT_ROI_TUNING_VALUES["expression_bbox_expand_ratio_x"])
    expression_bbox_expand_ratio_y: float = float(DEFAULT_ROI_TUNING_VALUES["expression_bbox_expand_ratio_y"])


@dataclass(slots=True)
class CameraConfig:
    device: str | None = None
    device_index: int = 0
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    pixel_format: str = "MJPG"
    warmup_frames: int = 5
    capture_timeout_ms: int = 3000
    interval_ms: int = 50
    max_frames: int | None = None
    emit_only_changes: bool = True
    save_frame: Path | None = None
    async_latest_frame: bool = True
    auto_exposure: int | None = None
    exposure_dynamic_framerate: int | None = None
    exposure_time_absolute: int | None = None
    gain: int | None = None
    brightness: int | None = None
    contrast: int | None = None
    saturation: int | None = None
    sharpness: int | None = None
    gamma: int | None = None
    white_balance_automatic: int | None = None
    white_balance_temperature: int | None = None
    focus_automatic_continuous: int | None = None
    focus_absolute: int | None = None
    backlight_compensation: int | None = None
    power_line_frequency: int | None = None


@dataclass(slots=True)
class PipelineConfig:
    dataset_dir: Path
    label_file: Path | None = None
    debug_dir: Path | None = None
    stop_after_stage: str | None = None
    debug_save_stages: bool = False
    ocr: OCRConfig = field(default_factory=OCRConfig)
    preprocess: PreprocessConfig = field(default_factory=PreprocessConfig)
