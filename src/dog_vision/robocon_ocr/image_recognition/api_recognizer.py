from __future__ import annotations

import base64
import io
import os
import threading
import time
from dataclasses import dataclass

import requests
from PIL import Image

from robocon_ocr.config import OCRConfig
from robocon_ocr.image_recognition.base import OCRResult


# ---------------------------------------------------------------------------
# 提供商注册表 — 后续加新模型只需在这里追加条目
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _Provider:
    """描述一个 API 提供商的端点、认证方式、默认模型。"""

    key: str
    default_base_url: str
    default_model: str
    env_prefix: str  # 读取 {PREFIX}_API_KEY / {PREFIX}_API_BASE_URL / {PREFIX}_MODEL
    auth_header: str = "Authorization"
    auth_template: str = "Bearer {api_key}"


# 注册的提供商（OpenAI 兼容协议）
_PROVIDERS: dict[str, _Provider] = {
    "moonshot": _Provider(
        key="moonshot",
        default_base_url="https://api.moonshot.cn/v1",
        default_model="moonshot-v1-8k-vision-preview",
        env_prefix="MOONSHOT",
    ),
    "openai": _Provider(
        key="openai",
        default_base_url="https://api.openai.com/v1",
        default_model="gpt-4o-mini",
        env_prefix="OPENAI",
    ),
    "qwen": _Provider(
        key="qwen",
        default_base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        default_model="qwen-vl-max",
        env_prefix="QWEN",
    ),
    "gemini": _Provider(
        key="gemini",
        default_base_url="https://generativelanguage.googleapis.com/v1beta",
        default_model="gemini-2.5-flash",
        env_prefix="GEMINI",
    ),
    "custom": _Provider(
        key="custom",
        default_base_url="",
        default_model="",
        env_prefix="CUSTOM",
    ),
}

# ---------------------------------------------------------------------------
# 识别 Prompt
# ---------------------------------------------------------------------------

_SYSTEM_PROMPT = (
    "You are a math expression OCR recognizer. "
    "Read the math expression in the image and output ONLY the expression itself. "
    "Use standard symbols: + - × ÷ = ( ). "
    "Do NOT include any explanations, prefixes, or markdown formatting. "
    'If you cannot read it confidently, output exactly: <UNKNOWN>'
)

_USER_PROMPT = "Recognize the math expression in this image."

# ---------------------------------------------------------------------------
# API 异常
# ---------------------------------------------------------------------------


class APIError(Exception):
    """联网大模型 API 调用失败。"""


# ---------------------------------------------------------------------------
# 识别器实现
# ---------------------------------------------------------------------------


class APIMathRecognizer:
    """多提供商联网大模型识别器。

    支持同步 (dataset 模式) 和非阻塞 (camera 模式) 两种调用路径。

    提供商切换：修改 ``api_provider`` 配置项即可，例如 ``moonshot`` →
    ``openai`` → ``qwen``。每个提供商的默认端点和模型在 ``_PROVIDERS``
    中注册，可通过环境变量覆盖。
    """

    backend_name: str = "api"
    supports_fallback_variants: bool = False
    is_async: bool = True  # 支持 submit/poll 非阻塞调用路径

    # 图片上传前缩放到此尺寸以内，省带宽降延迟
    _MAX_IMAGE_DIM = 768
    _JPEG_QUALITY = 85

    def __init__(self, config: OCRConfig) -> None:
        provider = _PROVIDERS.get(config.api_provider)
        if provider is None:
            raise ValueError(
                f"Unknown API provider: {config.api_provider}. "
                f"Available: {', '.join(_PROVIDERS)}"
            )

        self._provider = provider

        # 配置 → 环境变量 → 默认值 的优先级链
        self._api_key = config.api_key or _env(f"{provider.env_prefix}_API_KEY")
        self._base_url = config.api_base_url or _env(f"{provider.env_prefix}_API_BASE_URL") or provider.default_base_url
        self._model = config.api_model or _env(f"{provider.env_prefix}_MODEL") or provider.default_model
        self._timeout = config.api_timeout
        self._max_tokens = config.api_max_tokens

        # 非阻塞调用状态
        self._lock = threading.Lock()
        self._pending: bool = False
        self._latest_result: OCRResult | None = None
        self._latest_error: str | None = None
        self._cached_result: OCRResult | None = None  # 上次成功结果，pending 时继续展示
        self._generation: int = 0
        self._last_submit_time: float = 0.0
        self._min_interval: float = 3.0  # 两次 API 调用的最小间隔（秒）

    # ------------------------------------------------------------------
    # 同步接口 — dataset / 单张图像模式
    # ------------------------------------------------------------------

    def warmup(self) -> None:
        pass  # 云 API 不需要预热，但满足协议

    def recognize(self, image: Image.Image) -> OCRResult:
        """同步识别，阻塞等待 API 返回。用于 dataset 模式。"""
        try:
            text = self._call_api(image)
        except APIError as exc:
            return OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                error=str(exc),
                backend=self.backend_name,
            )
        return self._text_to_result(text)

    # ------------------------------------------------------------------
    # 非阻塞接口 — camera 模式
    # ------------------------------------------------------------------

    def submit(self, image: Image.Image) -> bool:
        """提交识别请求（不阻塞）。受 _min_interval 冷却限制，保证调用频率。"""
        with self._lock:
            now = time.monotonic()
            if now - self._last_submit_time < self._min_interval:
                return False
            self._pending = True
            self._last_submit_time = now
            self._latest_result = None
            self._latest_error = None
            self._generation += 1
            gen = self._generation

        t = threading.Thread(target=self._run_request, args=(image, gen), daemon=True)
        t.start()
        return True

    def poll(self) -> OCRResult | None:
        """返回已完成的结果并消费（清空），若仍在等待中则返回 None。"""
        with self._lock:
            if self._pending:
                return None
            err = self._latest_error
            result = self._latest_result
            self._latest_result = None
            self._latest_error = None
        if err is not None:
            return OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                error=err,
                backend=self.backend_name,
            )
        return result

    def has_pending(self) -> bool:
        with self._lock:
            return self._pending

    @property
    def status(self) -> str:
        """返回当前状态描述，供调试面板显示。"""
        with self._lock:
            if self._pending:
                elapsed = time.monotonic() - self._last_submit_time
                return f"waiting {elapsed:.1f}s"
            if self._latest_result is not None:
                return "ready"
            if self._latest_error is not None:
                return f"error: {self._latest_error[:60]}"
            return "idle"

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _run_request(self, image: Image.Image, generation: int) -> None:
        try:
            text = self._call_api(image)
            result = self._text_to_result(text)
            with self._lock:
                if generation != self._generation:
                    return  # 已被更新的请求取代，丢弃过期结果
                self._latest_result = result
                self._cached_result = result if not result.error else self._cached_result
                self._latest_error = None
                self._pending = False
        except APIError as exc:
            with self._lock:
                if generation != self._generation:
                    return
                self._latest_result = None
                self._latest_error = str(exc)
                self._pending = False

    def _call_api(self, image: Image.Image) -> str:
        if self._provider.key == "gemini":
            return self._call_gemini(image)
        return self._call_openai_compatible(image)

    def _call_openai_compatible(self, image: Image.Image) -> str:
        """OpenAI 兼容的 /v1/chat/completions 协议。

        Moonshot、OpenAI、Qwen 等均通过此路径。
        """
        if not self._api_key:
            raise APIError(f"API key not configured for provider '{self._provider.key}'")
        if not self._base_url:
            raise APIError(f"API base URL not configured for provider '{self._provider.key}'")

        image_b64 = self._encode_image(image)

        url = f"{self._base_url.rstrip('/')}/chat/completions"
        headers = {
            "Content-Type": "application/json",
            self._provider.auth_header: self._provider.auth_template.format(api_key=self._api_key),
        }
        # K2 系列模型要求 temperature=1 且只支持单条 user 消息
        is_k2 = self._model.startswith("kimi-k2")
        user_text = f"{_SYSTEM_PROMPT}\n\n{_USER_PROMPT}" if is_k2 else _USER_PROMPT
        messages: list[dict] = []
        if not is_k2:
            messages.append({"role": "system", "content": _SYSTEM_PROMPT})
        messages.append({
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                },
                {"type": "text", "text": user_text},
            ],
        })

        payload: dict = {
            "model": self._model,
            "messages": messages,
            "max_tokens": self._max_tokens,
            "temperature": 1.0 if is_k2 else 0.0,
        }

        try:
            resp = requests.post(url, headers=headers, json=payload, timeout=self._timeout)
        except requests.Timeout:
            raise APIError(f"API request timed out after {self._timeout}s")
        except requests.RequestException as exc:
            raise APIError(f"API request failed: {exc}") from exc

        if not resp.ok:
            detail = resp.text[:500]
            raise APIError(f"API returned {resp.status_code}: {detail}")

        try:
            body = resp.json()
            return body["choices"][0]["message"]["content"].strip()
        except (KeyError, IndexError, ValueError) as exc:
            raise APIError(f"Unexpected API response format: {exc}") from exc

    def _call_gemini(self, image: Image.Image) -> str:
        """Gemini generateContent API（非 OpenAI 兼容协议）。"""
        if not self._api_key:
            raise APIError("Gemini API key not configured")

        image_b64 = self._encode_image(image)

        url = f"{self._base_url.rstrip('/')}/models/{self._model}:generateContent?key={self._api_key}"
        payload = {
            "contents": [
                {
                    "parts": [
                        {"text": f"{_SYSTEM_PROMPT}\n\n{_USER_PROMPT}"},
                        {"inlineData": {"mimeType": "image/jpeg", "data": image_b64}},
                    ]
                }
            ],
            "generationConfig": {
                "maxOutputTokens": self._max_tokens,
                "temperature": 0.0,
            },
        }

        try:
            resp = requests.post(url, json=payload, timeout=self._timeout)
        except requests.Timeout:
            raise APIError(f"Gemini request timed out after {self._timeout}s")
        except requests.RequestException as exc:
            raise APIError(f"Gemini request failed: {exc}") from exc

        if not resp.ok:
            detail = resp.text[:500]
            raise APIError(f"Gemini returned {resp.status_code}: {detail}")

        try:
            body = resp.json()
            return body["candidates"][0]["content"]["parts"][0]["text"].strip()
        except (KeyError, IndexError, ValueError) as exc:
            raise APIError(f"Unexpected Gemini response format: {exc}") from exc

    @staticmethod
    def _text_to_result(text: str) -> OCRResult:
        cleaned = text.strip()
        if not cleaned or cleaned.upper() == "<UNKNOWN>":
            return OCRResult(
                raw_text="",
                confidence=0.0,
                lines=[],
                error="model could not recognize the expression",
                backend="api",
            )
        return OCRResult(
            raw_text=cleaned,
            confidence=1.0,  # 大模型不提供 token 级置信度，默认为 1
            lines=[cleaned],
            backend="api",
        )

    def _encode_image(self, image: Image.Image) -> str:
        """将 PIL Image 缩放到合理尺寸后编码为 base64 JPEG 字符串。"""
        w, h = image.size
        if max(w, h) > self._MAX_IMAGE_DIM:
            scale = self._MAX_IMAGE_DIM / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)), Image.LANCZOS)

        buf = io.BytesIO()
        # 确保 RGB 模式（二值化图可能是 L 模式）
        if image.mode != "RGB":
            image = image.convert("RGB")
        image.save(buf, format="JPEG", quality=self._JPEG_QUALITY)
        return base64.b64encode(buf.getvalue()).decode("ascii")


def _env(name: str) -> str:
    return os.getenv(name, "")
