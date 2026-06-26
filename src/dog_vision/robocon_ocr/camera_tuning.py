from __future__ import annotations

from robocon_ocr.config import CameraConfig


# 这份文件专门用于“电子显示屏中的白色题板 + 黑色题目 OCR”场景调参。
# 目标不是把画面调得更鲜艳，而是让：
# 1. 白色题板不要过曝发光
# 2. 黑色字符边缘清晰稳定
# 3. 自动曝光 / 自动白平衡 / 自动对焦不要来回漂移
# 4. 后续白色题板检测、二值化和 OCR 结果尽量稳定
#
# 推荐调参顺序：
# 1. 先调 exposure_time_absolute
# 2. 再调 focus_absolute
# 3. 再调 contrast
# 4. 最后才考虑 gain 和 brightness


DEFAULT_CAMERA_TUNING = CameraConfig(
    # 主项目当前默认使用第一个 USB 摄像头。
    device="/dev/video0",

    # 旧 CLI 兼容字段：未显式提供 device 时使用 /dev/video{device_index}。
    device_index=0,

    # 采集分辨率：1280x720 足够覆盖题板细节，同时运算开销可控。
    width=1280,
    height=720,

    # 帧率：30fps 是稳定 OCR 的优先选择。
    # 如果后续现场确认曝光足够、CPU 余量也够，再考虑提升。
    fps=30.0,

    # 固定为 MJPG。
    # 这是当前方案的硬要求，不再推荐 YUYV 分支。
    pixel_format="MJPG",

    # 预热帧数：给摄像头和驱动一点时间进入稳定状态。
    warmup_frames=5,
    capture_timeout_ms=3000,
    interval_ms=0,
    max_frames=None,
    emit_only_changes=True,
    save_frame=None,
    async_latest_frame=True,

    # 自动曝光：1 通常表示手动曝光模式。
    # 如果你发现 exposure_time_absolute 总是 inactive，先确认这里不是 3。
    auto_exposure=1,

    # 关闭“曝光带动帧率变化”。
    # 对 OCR 来说，我们更希望亮度稳定，而不是暗了就自动降帧。
    exposure_dynamic_framerate=0,

    # 手动曝光时间：这是最优先调的参数。
    # 白色题板发光、黑字发灰：减小它。
    # 整体太暗、黑字断裂：增大它。
    # 当前白底黑字屏幕场景，从 80 起步通常比较稳。
    exposure_time_absolute=200,

    # 增益：尽量保持低。
    # 增益会放大噪声，让字符边缘变脏。
    # 只有曝光再增大会拖慢或过曝时，才小幅增加。
    gain=0,

    # 亮度：保持中性，不主动把整张图抬亮。
    # 亮度过高常见现象是白题板变大片高亮，导致二值化更难分。
    brightness=0,

    # 对比度：决定黑白分离度。
    # OCR 二值图里字符边界发虚就加一点；笔画开始断裂就回退一点。
    contrast=20,

    # 饱和度：OCR 不依赖颜色，适度降低可以减少颜色噪声干扰。
    saturation=32,

    # 锐度：略高于默认，帮助字符边缘更明确。
    # 过高会产生假边缘和噪点，导致二值图变脏。
    sharpness=4,

    # gamma：先保持中性基线。
    # 如果你发现中间灰阶被压坏，再考虑微调，但它不是首要参数。
    gamma=100,

    # 关闭自动白平衡，避免画面随环境漂色。
    white_balance_automatic=0,

    # 手动白平衡温度：主要为稳定灰度分布，不追求肉眼观感。
    # 画面偏蓝可调高一些，偏黄可调低一些。
    white_balance_temperature=4600,

    # 关闭连续自动对焦。
    # 自动对焦会让字符边缘时不时“呼吸”，对 OCR 很伤。
    focus_automatic_continuous=0,

    # 手动焦距：第二优先调参项。
    # 用调试窗口观察黑字边缘，最锐利但不过分闪烁的位置通常最好。
    focus_absolute=190,

    # 背光补偿：识别屏幕时通常建议关掉，避免驱动自作主张提亮暗部。
    backlight_compensation=0,

    # 工频：1 代表 50Hz，适合当前环境。
    # 如果现场出现滚动条纹或闪烁，再检查这里和显示屏刷新关系。
    power_line_frequency=1,
)
