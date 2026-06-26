from __future__ import annotations

from pathlib import Path
from typing import Iterable


class CaptureSource:
    """视觉采集层统一接口，后续可扩展到摄像头、视频流或共享内存。"""

    def frames(self) -> Iterable[Path]:
        raise NotImplementedError

