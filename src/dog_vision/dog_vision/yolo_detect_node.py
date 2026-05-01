#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from dog_bringup.msg import Detection2D, Detection2DArray
from rclpy.node import Node


DEFAULT_CONFIG = {
    'model': '/home/angela/YOLO_ALL/Runs/task_2/train_task2/weights/best.pt',
    'mode': 'auto',
    'data': '/home/angela/YOLO_ALL/Dataset/task_2/yolo/data.yaml',
    'device': '/dev/video2',
    'width': 1920,
    'height': 1080,
    'fps': 30.0,
    'pixel_format': 'MJPG',
    'imgsz': 640,
    'conf': 0.6,
    'iou': 0.45,
    'show_image': False,
    'brightness': 0,
    'contrast': 32,
    'saturation': 64,
    'hue': 0,
    'white_balance_automatic': 1,
    'gamma': 100,
    'gain': 0,
    'power_line_frequency': 2,
    'white_balance_temperature': 4600,
    'sharpness': 3,
    'backlight_compensation': 1,
    'auto_exposure': 3,
    'exposure_time_absolute': 157,
    'exposure_dynamic_framerate': 1,
    'pan_absolute': 0,
    'tilt_absolute': 0,
    'focus_absolute': 170,
    'focus_automatic_continuous': 1,
    'zoom_absolute': 0,
}


def video_index_from_device(device: str):
    stripped = str(device).strip()
    if stripped.startswith('/dev/video'):
        return int(stripped.replace('/dev/video', ''))
    return device


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def set_v4l2_control(device: str, name: str, value: int, logger) -> None:
    result = subprocess.run(
        ['v4l2-ctl', '-d', device, '-c', f'{name}={value}'],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        logger.warning(f'Failed to set {name}={value}: {result.stderr.strip()}')


def set_v4l2_format(device: str, width: int, height: int, pixel_format: str, logger) -> None:
    result = subprocess.run(
        ['v4l2-ctl', '-d', device, f'--set-fmt-video=width={width},height={height},pixelformat={pixel_format}'],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        logger.warning(f'Failed to set camera format: {result.stderr.strip()}')


def set_v4l2_fps(device: str, fps: float, logger) -> None:
    result = subprocess.run(
        ['v4l2-ctl', '-d', device, f'--set-parm={fps}'],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        logger.warning(f'Failed to set FPS={fps}: {result.stderr.strip()}')


def configure_camera(cfg: dict[str, Any], logger) -> None:
    set_v4l2_format(cfg['device'], cfg['width'], cfg['height'], cfg['pixel_format'], logger)
    set_v4l2_fps(cfg['device'], cfg['fps'], logger)

    for control_name in (
        'white_balance_automatic',
        'auto_exposure',
        'focus_automatic_continuous',
        'brightness',
        'contrast',
        'saturation',
        'hue',
        'gamma',
        'gain',
        'power_line_frequency',
        'white_balance_temperature',
        'sharpness',
        'backlight_compensation',
        'exposure_time_absolute',
        'exposure_dynamic_framerate',
        'pan_absolute',
        'tilt_absolute',
        'focus_absolute',
        'zoom_absolute',
    ):
        value = cfg.get(control_name)
        if value is not None:
            set_v4l2_control(cfg['device'], control_name, int(value), logger)


def resolve_model_path(model_path: str) -> Path:
    path = Path(model_path).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.exists():
        raise FileNotFoundError(f'模型文件不存在: {path}')
    return path


def resolve_mode(model_path: Path, mode: str) -> str:
    suffix = model_path.suffix.lower()
    if mode == 'onnx' and suffix != '.onnx':
        raise RuntimeError(f'--mode onnx 需要 .onnx 模型文件，当前是: {model_path}')
    if mode == 'yolo' and suffix == '.onnx':
        raise RuntimeError(f'--mode yolo 需要 .pt 模型文件，当前是: {model_path}')
    if mode != 'auto':
        return mode
    return 'onnx' if suffix == '.onnx' else 'yolo'


def load_class_names(data_yaml: str | None, fallback_names: dict[int, str] | None = None) -> dict[int, str]:
    if fallback_names:
        return {int(k): str(v) for k, v in fallback_names.items()}
    if not data_yaml:
        return {}

    path = Path(data_yaml).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.exists():
        return {}

    with path.open('r', encoding='utf-8') as file_obj:
        data = yaml.safe_load(file_obj) or {}
    names = data.get('names', {})
    if isinstance(names, list):
        return {i: str(name) for i, name in enumerate(names)}
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}
    return {}


def color_for_class(class_id: int) -> tuple[int, int, int]:
    palette = [
        (0, 255, 0),
        (0, 200, 255),
        (255, 180, 0),
        (255, 0, 180),
        (180, 255, 0),
        (0, 180, 255),
        (255, 0, 0),
        (0, 0, 255),
    ]
    return palette[class_id % len(palette)]


def draw_detections(
    frame,
    detections: list[tuple[int, float, tuple[int, int, int, int]]],
    names: dict[int, str],
) -> None:
    for class_id, conf, (x1, y1, x2, y2) in detections:
        color = color_for_class(class_id)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{names.get(class_id, str(class_id))} {conf:.2f}"
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )


class YoloPtDetector:
    def __init__(self, model_path: Path, cfg: dict[str, Any]):
        from ultralytics import YOLO

        self.model = YOLO(str(model_path))
        self.cfg = cfg
        self.names = load_class_names(cfg['data'], getattr(self.model, 'names', None))

    def infer(self, frame) -> list[tuple[int, float, tuple[int, int, int, int]]]:
        results = self.model.predict(
            source=frame,
            imgsz=int(self.cfg['imgsz']),
            conf=float(self.cfg['conf']),
            iou=float(self.cfg['iou']),
            verbose=False,
        )
        detections: list[tuple[int, float, tuple[int, int, int, int]]] = []
        if not results:
            return detections
        result = results[0]
        if result.boxes is None:
            return detections
        for box in result.boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy()
            class_id = int(box.cls[0].detach().cpu().item())
            conf = float(box.conf[0].detach().cpu().item())
            x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
            detections.append((class_id, conf, (x1, y1, x2, y2)))
        return detections


class OnnxDetector:
    def __init__(self, model_path: Path, cfg: dict[str, Any]):
        import onnxruntime as ort

        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        available = ort.get_available_providers()
        providers = [provider for provider in providers if provider in available]
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        self.cfg = cfg
        self.names = load_class_names(cfg['data'])

    @staticmethod
    def _letterbox(frame, imgsz: int):
        h, w = frame.shape[:2]
        ratio = min(imgsz / w, imgsz / h)
        new_w = int(round(w * ratio))
        new_h = int(round(h * ratio))
        pad_w = imgsz - new_w
        pad_h = imgsz - new_h
        left = pad_w // 2
        top = pad_h // 2
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
        canvas[top: top + new_h, left: left + new_w] = resized
        return canvas, ratio, left, top

    @staticmethod
    def _nms(detections: list[tuple[int, float, tuple[int, int, int, int]]], iou_threshold: float):
        if not detections:
            return []
        boxes = [[x1, y1, x2 - x1 + 1, y2 - y1 + 1] for _, _, (x1, y1, x2, y2) in detections]
        scores = [conf for _, conf, _ in detections]
        indices = cv2.dnn.NMSBoxes(boxes, scores, score_threshold=0.0, nms_threshold=iou_threshold)
        if len(indices) == 0:
            return []
        flat_indices = np.array(indices).reshape(-1).tolist()
        return [detections[i] for i in flat_indices]

    def infer(self, frame) -> list[tuple[int, float, tuple[int, int, int, int]]]:
        imgsz = int(self.cfg['imgsz'])
        conf_threshold = float(self.cfg['conf'])
        iou_threshold = float(self.cfg['iou'])
        h, w = frame.shape[:2]
        image, ratio, pad_x, pad_y = self._letterbox(frame, imgsz)
        blob = image[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        blob = np.expand_dims(blob, axis=0)

        outputs = self.session.run(self.output_names, {self.input_name: blob})
        pred = np.squeeze(outputs[0])
        if pred.ndim != 2:
            return []
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T

        detections: list[tuple[int, float, tuple[int, int, int, int]]] = []
        for row in pred:
            if row.shape[0] < 5:
                continue
            scores = row[4:]
            class_id = int(np.argmax(scores))
            conf = float(scores[class_id])
            if conf < conf_threshold:
                continue

            cx, cy, bw, bh = [float(v) for v in row[:4]]
            x1 = int(round((cx - bw / 2.0 - pad_x) / ratio))
            y1 = int(round((cy - bh / 2.0 - pad_y) / ratio))
            x2 = int(round((cx + bw / 2.0 - pad_x) / ratio))
            y2 = int(round((cy + bh / 2.0 - pad_y) / ratio))
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(0, min(x2, w - 1))
            y2 = max(0, min(y2, h - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append((class_id, conf, (x1, y1, x2, y2)))

        return self._nms(detections, iou_threshold)


def build_detector(model_path: Path, cfg: dict[str, Any]):
    mode = resolve_mode(model_path, str(cfg['mode']))
    if mode == 'onnx':
        return OnnxDetector(model_path, cfg), mode
    return YoloPtDetector(model_path, cfg), mode


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')
        self.cfg = {}
        for key, default_value in DEFAULT_CONFIG.items():
            self.declare_parameter(key, default_value)
            self.cfg[key] = self.get_parameter(key).value

        self.publisher_ = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.cap = None
        self.detector = None
        self.mode = ''
        self.last_time = time.perf_counter()
        self.fps = 0.0
        self.frame_timer = None

        model_path = resolve_model_path(str(self.cfg['model']))
        self.detector, self.mode = build_detector(model_path, self.cfg)
        self.get_logger().info(f'模型已加载: {model_path}')
        self.get_logger().info(f'推理模式: {self.mode}')

        configure_camera(self.cfg, self.get_logger())
        self.cap = cv2.VideoCapture(video_index_from_device(self.cfg['device']), cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头: {self.cfg['device']}")

        period = 1.0 / max(float(self.cfg['fps']), 1.0)
        self.frame_timer = self.create_timer(period, self.process_frame)

    def process_frame(self) -> None:
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('读取画面失败。')
            return

        detections = self.detector.infer(frame)
        message = Detection2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.cfg['device'])

        for class_id, conf, (x1, y1, x2, y2) in detections:
            item = Detection2D()
            item.class_id = int(class_id)
            item.class_name = self.detector.names.get(class_id, str(class_id))
            item.confidence = float(conf)
            item.x1 = int(x1)
            item.y1 = int(y1)
            item.x2 = int(x2)
            item.y2 = int(y2)
            message.detections.append(item)

        self.publisher_.publish(message)

        now = time.perf_counter()
        dt = max(now - self.last_time, 1e-6)
        self.last_time = now
        self.fps = self.fps * 0.9 + (1.0 / dt) * 0.1 if self.fps > 0 else 1.0 / dt

        if as_bool(self.cfg['show_image']):
            draw_detections(frame, detections, self.detector.names)
            cv2.putText(
                frame,
                f'FPS {self.fps:.1f}',
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow('YOLO Camera Inference', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.frame_timer is not None:
            self.frame_timer.cancel()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if as_bool(self.cfg.get('show_image')):
            cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = YoloDetectNode()
        rclpy.spin(node)
    except (FileNotFoundError, RuntimeError, ImportError) as error:
        logger = rclpy.logging.get_logger('yolo_detect_node')
        logger.error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
