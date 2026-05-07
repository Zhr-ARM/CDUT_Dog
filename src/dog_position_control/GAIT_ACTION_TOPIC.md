# Gait Action Topic

本文档说明如何通过 ROS 2 话题切换机器狗动作。

## 接口

控制器订阅一个动作话题:

```text
/gait_action
```

消息类型:

```text
std_msgs/msg/String
```

只需要填写一个字段:

```text
data
```

真机配置文件中对应参数在 `config/gait_controller_real_stand.yaml`:

```yaml
gait_action_topic: /gait_action
stand_only: true
enable_auto_sequence: false
```

含义是: 机器狗启动后先起立并保持站立，等待外部节点向 `/gait_action` 发布动作命令。

## 支持的动作

| data | 动作 |
| --- | --- |
| `stand` | 站立保持 |
| `crouch` | 下蹲/低趴保持 |
| `step` | 原地踏步 |
| `forward` | 前进 |
| `backward` | 后退 |
| `lateral_left` | 机身不转，向左横移 |
| `lateral_right` | 机身不转，向右横移 |
| `turn_left` | 左转 |
| `turn_right` | 右转 |
| `forward_left` | 机身不转，向左前斜向平移 |
| `forward_right` | 机身不转，向右前斜向平移 |
| `backward_left` | 机身不转，向左后斜向平移 |
| `backward_right` | 机身不转，向右后斜向平移 |
| `auto` | 切回自动演示序列 |

也支持一些别名:

| 别名 | 等价动作 |
| --- | --- |
| `stop`, `idle`, `hold` | `stand` |
| `squat`, `down`, `lie_down`, `lower` | `crouch` |
| `in_place`, `in_place_step`, `march` | `step` |
| `fwd` | `forward` |
| `back`, `reverse` | `backward` |
| `left` | `turn_left` |
| `right` | `turn_right` |
| `strafe_left`, `move_left`, `side_left` | `lateral_left` |
| `strafe_right`, `move_right`, `side_right` | `lateral_right` |
| `left_forward`, `diagonal_left`, `diag_left` | `forward_left` |
| `right_forward`, `diagonal_right`, `diag_right` | `forward_right` |
| `left_backward`, `back_left`, `reverse_left` | `backward_left` |
| `right_backward`, `back_right`, `reverse_right` | `backward_right` |
| `sequence`, `auto_sequence` | `auto` |

输入会自动转成小写，并把空格、`-`、`/` 转成 `_`。例如 `turn-left` 和 `turn left` 都会被识别成 `turn_left`。

## 命令行测试

先启动真机:

```bash
ros2 launch dog_bringup real.launch.py launch_simulation:=false \
  gait_param_file:=/home/zhr/robot_dog/src/dog_position_control/config/gait_controller_real_stand.yaml
```

发布一次动作命令:

```bash
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: forward}"
```

常用测试命令:

```bash
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: stand}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: crouch}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: step}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: forward}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: backward}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: lateral_left}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: lateral_right}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: turn_left}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: turn_right}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: forward_left}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: forward_right}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: backward_left}"
ros2 topic pub --once /gait_action std_msgs/msg/String "{data: backward_right}"
```

## 外部节点发布示例

Python:

```python
from std_msgs.msg import String

msg = String()
msg.data = "forward"
publisher.publish(msg)
```

C++:

```cpp
#include "std_msgs/msg/string.hpp"

auto msg = std_msgs::msg::String();
msg.data = "forward";
publisher->publish(msg);
```

## 控制逻辑说明

收到 `/gait_action` 后，控制器会进入手动动作模式，并关闭自动演示序列。

- `stand`: 切到站立保持。
- `crouch`: 切到站立保持状态，但站姿目标使用 `startup_crouch_*_targets` 低趴目标。
- `step`, `forward`, `backward`, `turn_left`, `turn_right`: 切到 walk 状态，并使用对应动作的 direct joint 摆动参数。
- `lateral_left`, `lateral_right`: 只叠加 HAA 横向摆动，实现机身不转的左右横移。
- `forward_left`, `forward_right`, `backward_left`, `backward_right`: 在前后 HFE 摆动基础上叠加小幅 HAA 横向摆动，实现机身不转的斜向平移。
- `auto`: 重新启用自动演示序列。

斜向平移的横向幅度由 `auto_sequence_diagonal_haa_amplitude` 控制，逐腿方向由
`auto_sequence_diagonal_haa_signs` 控制。纯横移单独使用
`auto_sequence_lateral_haa_amplitude` 和 `auto_sequence_lateral_haa_signs`，避免横移
和斜向共用一套 HAA 标定导致看起来像原地扭转。第一次真机测试建议保持小幅度，并
先确认 `forward_left`、`lateral_left` 的实际方向是否正确。

如果机器狗还在起立阶段，动作命令会被记住；起立完成后再进入对应动作。

## 相关代码位置

- 话题参数声明: `src/quadruped_gait_controller.cpp`
- 订阅创建: `src/quadruped_gait_controller.cpp`
- 回调入口: `gait_action_callback`
- 动作字符串解析: `parse_gait_action`
- 状态切换: `update_gait_action_command_state` 和 `apply_gait_action_command`
- 真机配置: `config/gait_controller_real_stand.yaml`
