#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path


BACKEND_CHOICES = ("api", "onnx")


def _load_env_file(path: str) -> None:
    env_path = Path(path).expanduser()
    if not env_path.is_file():
        return
    with env_path.open(encoding="utf-8") as file_obj:
        for line in file_obj:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            if key and key not in os.environ:
                os.environ[key] = value.strip()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run robocon_ocr as a JSON-line worker.")
    parser.add_argument("--backend", choices=BACKEND_CHOICES, default="api")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--pixel-format", default="MJPG")
    parser.add_argument("--show-image", default="false")
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--capture-timeout-ms", type=int, default=3000)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--filter-consensus", type=int, default=3)
    parser.add_argument("--onnx-model-path", default="")
    parser.add_argument("--onnx-dict-path", default="")
    parser.add_argument("--api-provider", default="moonshot")
    parser.add_argument("--api-model", default="")
    parser.add_argument("--api-key", default="")
    parser.add_argument("--api-base-url", default="")
    parser.add_argument("--api-timeout", type=float, default=5.0)
    parser.add_argument("--api-max-tokens", type=int, default=128)
    parser.add_argument("--env-file", default="")
    return parser


def _emit(payload: dict[str, object]) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def _output_to_payload(output) -> dict[str, object]:
    return {
        "type": "result",
        "expression": output.expression,
        "answer": output.answer,
        "answer_mod_4": output.answer_mod_4,
        "is_valid": output.is_valid,
        "confidence": output.confidence,
        "backend": output.backend,
        "error": output.error or "",
        "created_at": time.time(),
    }


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.env_file:
        _load_env_file(args.env_file)

    from robocon_ocr import CameraRecognitionSession
    from robocon_ocr.config import CameraConfig, OCRConfig

    stop_event = threading.Event()
    session: CameraRecognitionSession | None = None

    def handle_signal(_signum, _frame) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    ocr_config = OCRConfig(
        backend=args.backend,
        api_provider=args.api_provider,
        api_model=args.api_model,
        api_key=args.api_key,
        api_base_url=args.api_base_url,
        api_timeout=args.api_timeout,
        api_max_tokens=args.api_max_tokens,
    )
    if args.onnx_model_path:
        ocr_config.onnx_model_path = args.onnx_model_path
    if args.onnx_dict_path:
        ocr_config.onnx_dict_path = args.onnx_dict_path
    camera_config = CameraConfig(
        device=args.device or None,
        device_index=args.device_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        pixel_format=args.pixel_format,
        warmup_frames=args.warmup_frames,
        capture_timeout_ms=args.capture_timeout_ms,
        max_frames=args.max_frames if args.max_frames > 0 else None,
    )

    def on_result(output) -> None:
        _emit(_output_to_payload(output))

    try:
        session = CameraRecognitionSession(
            backend=args.backend,
            on_result=on_result,
            filter_consensus=args.filter_consensus,
            camera_config=camera_config,
            ocr_config=ocr_config,
            show_image=str(args.show_image).strip().lower() in {"1", "true", "yes", "on"},
        )
        session.start_async()
        _emit({"type": "status", "status": "running", "backend": args.backend})

        exit_code = 0
        while not stop_event.is_set():
            thread = session._thread
            if thread is not None and not thread.is_alive():
                if session.error is not None:
                    print(f"ocr session failed: {session.error}", file=sys.stderr, flush=True)
                    exit_code = 1
                else:
                    exit_code = 0
                break
            time.sleep(0.1)
        return exit_code
    except Exception as exc:
        print(f"ocr worker failed: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if session is not None:
            session.stop()


if __name__ == "__main__":
    raise SystemExit(main())
