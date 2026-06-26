# dog_vision 视觉模块说明手册

本文说明 `dog_vision` 中 YOLO 与 OCR 的启动方式、摄像头切换方式、OCR 后端选择，以及识别结果的话题输出形式。

## 1. 模块组成

`dog_vision` 目前主要包含以下视觉节点：

- `yolo_detect_node`：YOLO 目标检测节点，使用机械臂摄像头发布目标框结果。
- `ocr_recognition_node`：OCR ROS2 桥接节点，按选择的后端启动 OCR 子进程，并发布算式识别结果。
- `camera_mode_mux_node`：摄像头模式切换节点，用 `off/yolo/ocr` 三种模式保证 YOLO 与 OCR 不会同时占用同一个 USB 摄像头。
- `target_selector_node`：订阅 YOLO 检测结果，选择目标编号并发布给后续任务。

YOLO 和 OCR 共用同一个摄像头，默认设备路径是：

```text
/dev/video0
```

如果现场设备号变化，可以启动时通过 `device:=/dev/video1` 或 bringup 中的 `camera_device:=/dev/video1` 指定。

## 2. 编译与环境加载

在主工作区根目录执行：

```bash
colcon build --packages-select dog_bringup dog_vision
source install/setup.bash
```

如果只修改本文档，不需要重新编译；如果修改了 launch、节点代码或消息定义，需要重新执行 `colcon build`。

## 3. 摄像头模式切换

YOLO 和 OCR 不应该同时运行。统一通过 `/vision/mode` 控制当前视觉模式：

```bash
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'yolo'}"
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'ocr'}"
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'off'}"
```

三种模式含义：

- `yolo`：开启 YOLO，关闭 OCR。
- `ocr`：开启 OCR，关闭 YOLO。
- `off`：YOLO 与 OCR 都关闭，并释放摄像头。

兼容旧接口：

```text
/arm_vision_mode/enabled  std_msgs/msg/Bool
```

旧 Bool 话题仍然可用：

- `true` 映射为 `yolo`
- `false` 映射为 `off`

当前实际生效模式可以查看：

```bash
ros2 topic echo /vision/active_mode
```

## 4. 启动 dog_vision

只启动视觉包：

```bash
ros2 launch dog_vision dog_vision.launch.py
```

默认行为：

- 启动 `camera_mode_mux_node`
- 启动 `yolo_detect_node`
- 不启动 `ocr_recognition_node`
- 初始模式为 `off`

如果需要同时把 YOLO 和 OCR 节点都启动起来，再通过模式话题切换谁占用摄像头：

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=true
```

启动后再选择模式：

```bash
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'yolo'}"
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'ocr'}"
```

## 5. YOLO 调用方式

启动 YOLO：

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=false
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'yolo'}"
```

常用 launch 参数：

```text
model         YOLO 模型路径，默认使用 dog_vision/models/task_2/best.onnx
data          YOLO data.yaml 路径，默认使用 dog_vision/config/task_2_data.yaml
mode          推理模式，默认 auto；当前项目只推荐使用 ONNX 模型，实际会走 onnx
device        摄像头设备路径，默认 /dev/video0
width         摄像头宽度，默认 1920
height        摄像头高度，默认 1080
fps           摄像头帧率，默认 30.0
pixel_format  摄像头格式，默认 MJPG
imgsz         YOLO 输入尺寸，默认 640
conf          置信度阈值，默认 0.6
iou           NMS IoU 阈值，默认 0.45
show_image    是否显示当前活动视觉模式的调试画面，默认 false
```

YOLO 当前默认且只允许使用 ONNX 模型。传入 `.pt` 模型或 `mode:=yolo` 会被节点拒绝，不会再使用 `ultralytics`。主 ROS Python 环境需要安装 ONNX 推理依赖：

```bash
python3 -m pip install --user -r src/dog_vision/requirements/yolo-onnx.txt
```

注意：启动时传 `show_image:=true` 后，只有当前模式真正打开摄像头时才会显示窗口。刚启动时默认是 `off`，需要再发布模式：

```bash
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'yolo'}"
```

示例：

```bash
ros2 launch dog_vision dog_vision.launch.py \
  launch_yolo:=true \
  launch_ocr:=false \
  model:=/home/cdutdog/CDUT_Dog/CDUT_Dog/src/dog_vision/models/task_2/best.onnx \
  conf:=0.6 \
  show_image:=false
```

YOLO 结果输出：

```text
/vision/detections  dog_bringup/msg/Detection2DArray
```

查看结果：

```bash
ros2 topic echo /vision/detections
```

消息结构：

```text
Detection2DArray
  std_msgs/Header header
  Detection2D[] detections

Detection2D
  int32 class_id
  string class_name
  float32 confidence
  int32 x1
  int32 y1
  int32 x2
  int32 y2
```

字段含义：

- `class_id`：类别 ID。
- `class_name`：类别名称。
- `confidence`：检测置信度。
- `x1/y1/x2/y2`：目标框在当前摄像头图像中的像素坐标。

## 6. OCR 调用方式

启动 OCR：

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=false launch_ocr:=true
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'ocr'}"
```

推荐实际使用方式是同时启动 YOLO 和 OCR 节点，然后只通过 `/vision/mode` 切换：

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=true show_image:=true
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'ocr'}"
```

关闭 OCR 并释放摄像头：

```bash
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'off'}"
```

OCR 常用 launch 参数：

```text
ocr_backend           OCR 后端，支持 api/onnx，默认 api
ocr_allow_api_fallback  是否允许 API 失败后回退到 ONNX，默认 false
ocr_width             OCR 摄像头宽度，默认 1280
ocr_height            OCR 摄像头高度，默认 720
ocr_filter_consensus  本地 OCR 连续帧一致性阈值，默认 3
onnx_python           ONNX OCR 虚拟环境 Python
api_python            联网 API OCR 使用的 Python
ocr_onnx_model_path   OCR ONNX 模型路径
ocr_onnx_dict_path    OCR 字典路径
ocr_env_file          API Key 环境变量文件，默认主工作区 .env
```

`show_image:=true` 时，OCR 模式会显示 `OCR Camera Recognition` 窗口，画面里会叠加 ROI 框、后端、原始识别文本、表达式和结果。

## 7. OCR 后端选择

通过 `ocr_backend` 显式选择后端。默认是联网 API 后端；如果选择 `api`，节点只跑 API；如果选择 `onnx`，节点只跑 ONNX。

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=true ocr_backend:=api
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=true ocr_backend:=onnx
```

只有显式设置下面这个参数时，API worker 失败后才会自动切到 ONNX：

```bash
ros2 launch dog_vision dog_vision.launch.py launch_yolo:=true launch_ocr:=true ocr_backend:=api ocr_allow_api_fallback:=true
```

两种后端说明：

| 后端 | 参数值 | 运行环境 | 说明 |
| --- | --- | --- | --- |
| 联网 API 后端 | `api` | `.venv-ocr-onnx/bin/python` | 默认首选，调用联网大模型或视觉 API，需要 API Key。 |
| ONNX 本地后端 | `onnx` | `.venv-ocr-onnx/bin/python` | 次选后端，使用本地 ONNX 模型，适合离线或 API 不可用时运行。 |

查看当前实际运行后端：

```bash
ros2 topic echo /vision/ocr_backend
```

指定 ONNX 虚拟环境 Python：

```bash
ros2 launch dog_vision dog_vision.launch.py \
  launch_yolo:=true \
  launch_ocr:=true \
  ocr_backend:=onnx \
  onnx_python:=/home/cdutdog/CDUT_Dog/CDUT_Dog/.venv-ocr-onnx/bin/python
```

PaddleOCR 原生模型后端已经禁用，`ocr_backend:=lightweight` 不再是合法选项。

联网 API 后端需要环境变量。模板文件在：

```text
src/dog_vision/.env.example
```

真实 Key 默认放在主工作区：

```text
/home/cdutdog/CDUT_Dog/CDUT_Dog/.env
```

也可以启动时显式指定：

```bash
ros2 launch dog_vision dog_vision.launch.py \
  launch_yolo:=true \
  launch_ocr:=true \
  ocr_backend:=api \
  ocr_env_file:=/home/cdutdog/CDUT_Dog/CDUT_Dog/.env
```

注意：真实 `.env` 不要提交到仓库。

## 8. OCR 结果输出

OCR 主要输出这些话题：

```text
/vision/ocr_result  dog_bringup/msg/OcrResult
/vision/ocr_mod4    std_msgs/msg/Int32
/vision/ocr_status  std_msgs/msg/String
/vision/ocr_backend std_msgs/msg/String
/voice_broadcast    std_msgs/msg/String
```

查看完整 OCR 结果：

```bash
ros2 topic echo /vision/ocr_result
```

`OcrResult` 消息结构：

```text
std_msgs/Header header
string expression
bool has_answer
int32 answer
int32 answer_mod_4
bool is_valid
float32 confidence
string backend
string error
```

字段含义：

- `expression`：识别出的算式字符串，例如 `11+17+14`。
- `has_answer`：是否成功解析出计算结果。
- `answer`：算式计算结果。
- `answer_mod_4`：`answer % 4` 的结果，供机器人任务逻辑直接使用。
- `is_valid`：本次 OCR 结果是否可信且可用。
- `confidence`：OCR 置信度，不同后端的置信度含义可能略有差异。
- `backend`：产生该结果的 OCR 后端，例如 `api` 或 `onnx`。
- `error`：错误信息。正常结果一般为空字符串。

`/vision/ocr_mod4` 只在 `is_valid=true` 且结果可用时发布：

```bash
ros2 topic echo /vision/ocr_mod4
```

同一时刻，OCR 节点也会把 `answer_mod_4` 作为字符串发布到 `/voice_broadcast`，
由 `voice_broadcast_node` 通过喇叭播报。常用 launch 入口在 `launch_ocr:=true`
时会自动启动该语音节点，并把 `idle_timeout` 设为 `0`，避免等待 OCR 结果时语音节点空闲退出。

OCR 状态查看：

```bash
ros2 topic echo /vision/ocr_status
```

常见状态：

```text
idle
starting:api
starting:onnx
running:api
running:onnx
stopping
stopped
error:api:...
error:onnx:...
```

## 9. dog_bringup 中的调用方式

`dog_bringup` 的任务 launch 也透传了视觉相关参数，可以从总启动入口直接开启 OCR。

示例：

```bash
ros2 launch dog_bringup target_perception.launch.py launch_yolo:=true launch_ocr:=true
```

或在真实导航任务中开启：

```bash
ros2 launch dog_bringup target_nav_real.launch.py launch_yolo:=true launch_ocr:=true ocr_backend:=api
```

启动后仍然通过 `/vision/mode` 切换当前占用摄像头的任务：

```bash
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'yolo'}"
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'ocr'}"
ros2 topic pub --once /vision/mode std_msgs/msg/String "{data: 'off'}"
```

## 10. 快速检查命令

查看有哪些视觉话题：

```bash
ros2 topic list | grep vision
```

查看 OCR 节点状态：

```bash
ros2 topic echo /vision/ocr_status
```

查看当前模式：

```bash
ros2 topic echo /vision/active_mode
```

查看摄像头设备：

```bash
ls -l /dev/video*
```

查看 launch 参数：

```bash
ros2 launch dog_vision dog_vision.launch.py --show-args
```

## 11. 常见问题

如果 YOLO 或 OCR 打不开摄像头，先确认当前模式是否正确，并确认另一个视觉任务已经停止：

```bash
ros2 topic echo /vision/active_mode
```

如果 OCR API 后端报 Key 相关错误，检查：

```text
ocr_backend:=api
ocr_env_file:=/home/cdutdog/CDUT_Dog/CDUT_Dog/.env
```

如果传入 `ocr_backend:=lightweight`，节点会拒绝该参数。当前允许的 OCR 后端只有：

```text
api
onnx
```

如果 ONNX 后端启动失败，检查 `.venv-ocr-onnx` 和模型文件：

```text
src/dog_vision/requirements/ocr-onnx.txt
src/dog_vision/models/ocr/PP-OCRv5_server_rec.onnx
src/dog_vision/models/ocr/dict.txt
```
