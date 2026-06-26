from __future__ import annotations

from PIL import Image


class FrameProcessor:
    """视觉处理层统一接口，后续可接透视矫正、去反光、去模糊等处理。"""

    def process(self, image: Image.Image) -> Image.Image:
        return image

