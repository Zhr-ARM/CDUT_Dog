# CDUT_Dog

CDUT_Dog 是一个面向仿生足式机器人任务赛的 ROS 2 工作空间，覆盖四足底盘、电机驱动、步态控制、手柄遥控、IMU、物资箱感知定位、机械臂 MoveIt 控制和吸盘执行器。

当前代码已经具备分模块调试能力，也提供了部分任务赛链路入口；完整的“识别题目 -> 选择物资箱 -> 搬运归位 -> 自动计时/恢复”的任务级状态机仍在开发中。

## 文档导航

- [任务赛规则与比赛流程](docs/task_competition_rules.md)：根据仓库根目录 `仿生足式规则V2.0.pdf` 整理，仅保留任务赛相关规则。
- [任务赛代码完成度与待办](docs/task_implementation_status.md)：按比赛链路梳理当前代码已完成和待完成的功能。
- [步态动作话题说明](src/dog_position_control/GAIT_ACTION_TOPIC.md)：`/gait_action` 可用动作与调试方式。
- [DeepRobotics 电机驱动说明](src/deep_motor_ros/Readme.md)：电机驱动包的独立说明。

## 工作空间结构

```text
CDUT_Dog/
├── README.md
├── 仿生足式规则V2.0.pdf
├── docs/
│   ├── task_competition_rules.md
│   └── task_implementation_status.md
└── src/
    ├── dog_bringup/             # 总启动入口、机器人模型、仿真场景、udev 规则、自定义消息
    ├── deep_motor_ros/          # DeepRobotics J60 电机 SocketCAN 驱动
    ├── dog_position_control/    # 四足站立、步态、/cmd_vel 和 /gait_action 控制
    ├── mit_controller/          # Gazebo/ros2_control 下的 MIT 风格关节控制器
    ├── dog_teleop/              # 手柄遥控，/joy -> /cmd_vel 或 /gait_action
    ├── dog_imu/                 # WitMotion IMU 串口/CAN 驱动
    ├── dog_lidar/               # 点云物资箱网格定位，发布目标箱位姿
    ├── dog_vision/              # YOLO 标签检测、目标选择、目标接近预览/控制
    ├── arm_model/               # 机械臂 URDF/Xacro
    ├── arm_movelt/              # 机械臂 MoveIt 配置与真机启动
    ├── arm_control/             # 机械臂硬件接口、MoveIt 规划服务、箱子动作序列
    └── suction_controller/      # 吸盘和放气阀继电器串口控制
```

## 主要能力

| 模块 | 当前能力 | 主要入口 |
|---|---|---|
| 四足底盘仿真 | Gazebo 加载狗模型、`mit_controller`、步态控制器和 RViz | `ros2 launch dog_bringup sim.launch.py` |
| 四足底盘真机 | 4 路 CAN、12 个 J60 关节、真机关节反馈、站立和保守步态 | `ros2 launch dog_bringup real.launch.py` |
| 手柄遥控 | Xbox/PS 手柄映射为前后、横移、转向、下蹲/站立等指令 | `ros2 launch dog_teleop dog_teleop.launch.py` |
| IMU | WitMotion normal/modbus/hmodbus/can/hcan 协议，发布 IMU/Mag/GPS | `ros2 launch dog_imu wit_imu.launch.py` |
| 物资箱点云定位 | 从点云中筛选 2 排 4 列箱体，发布选中箱 `PoseStamped` | `ros2 launch dog_bringup target_perception.launch.py` |
| 视觉标签检测 | YOLO `.pt` 或 `.onnx` 模型检测标签，发布 2D 检测结果 | `ros2 launch dog_vision dog_vision.launch.py` |
| 目标接近 | 根据目标箱位姿生成预览路径，可选择发布 `/cmd_vel` | `ros2 launch dog_bringup target_nav_real.launch.py` |
| 机械臂 | DM-J4340 硬件接口、MoveIt、规划服务、箱子吸取/抬起/放置动作 | `ros2 launch dog_bringup arm.launch.py` |
| 吸盘 | 继电器 1 控制吸附，继电器 2 脉冲放气释放 | `ros2 launch suction_controller suction_controller.launch.py` |
| 总启动 | 一次拉起底盘、机械臂、感知模块 | `ros2 launch dog_bringup all.launch.py` |

## 环境准备

建议环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- MoveIt 2
- SocketCAN、PCL、OpenCV、pyserial
- 可选：`ultralytics`、`onnxruntime`，用于 YOLO `.pt` 或 `.onnx` 推理

编译完成后，每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

仓库里的 `.envrc` 会加载 `.vscode/disable_conda.bash`，用于避免 Conda 环境污染 ROS 运行环境。

## udev 设备名

真机调试前建议安装稳定设备名规则：

```bash
sudo cp src/dog_bringup/config/99-serial-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

规则会创建这些语义设备名：

| 设备 | 名称 |
|---|---|
| IMU 串口 | `/dev/ttyimu` |
| 四足 CAN | `can_lf`、`can_lh`、`can_rf`、`can_rh` |
| 机械臂电机 USB-CAN | `/dev/arm_motor` |
| 吸盘继电器 | `/dev/arm_relay1` |
| 放气阀继电器 | `/dev/arm_relay2` |
| 机械臂相机 | `/dev/arm_camera` |

如果 USB 拓扑或设备 VID/PID 改变，需要同步修改 `src/dog_bringup/config/99-serial-can.rules` 和相关 YAML/launch 参数。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译某个包示例：

```bash
colcon build --symlink-install --packages-select dog_position_control
```

## 启动与调试

### Gazebo 仿真

```bash
ros2 launch dog_bringup sim.launch.py
```

关闭 RViz：

```bash
ros2 launch dog_bringup sim.launch.py launch_rviz:=false
```

只启动仿真但不启动步态控制器：

```bash
ros2 launch dog_bringup sim.launch.py launch_gait_controller:=false
```

### 四足真机

```bash
ros2 launch dog_bringup real.launch.py
```

常用开关：

```bash
ros2 launch dog_bringup real.launch.py \
  launch_simulation:=false \
  launch_teleop:=false \
  launch_imu:=true
```

真机关键配置：

- `src/deep_motor_ros/config/four_can_real_robot.yaml`
- `src/dog_position_control/config/gait_controller_real_stand.yaml`
- `src/dog_bringup/config/99-serial-can.rules`

### 手柄遥控

```bash
ros2 launch dog_teleop dog_teleop.launch.py
```

默认输出 `/cmd_vel`：

| 操作 | 效果 |
|---|---|
| 左摇杆前后 | 前进/后退 |
| 左摇杆左右 | 左右横移 |
| 右摇杆左右 | 原地转向 |
| A 键 | 立即站立/零速 |
| X/Y 键 | 下蹲/站立，见 `dog_teleop.yaml` |

命令行验证：

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### IMU

```bash
ros2 launch dog_imu wit_imu.launch.py type:=normal port:=/dev/ttyimu baud:=9600
```

查看数据：

```bash
ros2 topic echo /wit/imu
ros2 run dog_imu get_imu_rpy
```

### 感知和目标接近

只启动点云箱体定位、可选 YOLO 和目标选择：

```bash
ros2 launch dog_bringup target_perception.launch.py \
  cloud_topic:=/odin1/cloud_render \
  target_row:=0 \
  target_col:=1 \
  launch_yolo:=false \
  launch_target_selector:=false
```

启动真机目标接近预览。默认不会发布 `/cmd_vel`，适合先看 RViz 路径：

```bash
ros2 launch dog_bringup target_nav_real.launch.py publish_cmd_vel:=false
```

确认目标稳定后再允许它发布速度：

```bash
ros2 launch dog_bringup target_nav_real.launch.py \
  launch_real_control:=true \
  publish_cmd_vel:=true \
  launch_teleop:=false
```

相关话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/vision/detections` | `dog_bringup/msg/Detection2DArray` | YOLO 2D 检测结果 |
| `/vision/target_id` | `std_msgs/msg/Int32MultiArray` | 目标行列 `[row, col]` |
| `/navigation/target_pose` | `geometry_msgs/msg/PoseStamped` | 点云定位出的目标箱位姿 |
| `/navigation/preview_path` | `nav_msgs/msg/Path` | 目标接近预览路径 |
| `/navigation/nav_status` | `std_msgs/msg/String` | 目标接近状态 |

### 机械臂和吸盘

```bash
ros2 launch dog_bringup arm.launch.py
```

常用动作触发：

```bash
# 进入视觉观察姿态
ros2 topic pub --once /arm_vision_motion/start std_msgs/msg/Empty "{}"

# 完整箱子动作: 视觉姿态 -> 吸取 -> 抬起 -> 放置 -> 视觉姿态
ros2 topic pub --once /arm_box_motion/start std_msgs/msg/Empty "{}"

# 吸取、抬起、地面放置、叠放可以分段调试
ros2 topic pub --once /arm_box_motion/suction std_msgs/msg/Empty "{}"
ros2 topic pub --once /arm_box_motion/lift std_msgs/msg/Empty "{}"
ros2 topic pub --once /arm_box_motion/place_ground std_msgs/msg/Empty "{}"
ros2 topic pub --once /arm_box_motion/place_stack std_msgs/msg/Empty "{}"

# 回原位并关闭吸盘
ros2 topic pub --once /arm_home_motion/start std_msgs/msg/Empty "{}"
```

直接控制吸盘：

```bash
ros2 topic pub --once /suction/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /suction/enable std_msgs/msg/Bool "{data: false}"
```

### 总启动

```bash
ros2 launch dog_bringup all.launch.py
```

默认会启动底盘、机械臂和感知模块。比赛调试时建议先按模块验证，再使用总启动入口。

## 常用检查命令

```bash
ros2 topic echo /joint_states
ros2 topic echo /real/joint_states
ros2 topic echo /motor_cmd
ros2 topic echo /real/motor_feedback
ros2 topic echo /cmd_vel
ros2 topic echo /gait_status
ros2 topic echo /boxes/status
ros2 topic echo /navigation/nav_status
ros2 topic echo /wit/imu
```

## 关键配置

| 文件 | 作用 |
|---|---|
| `src/deep_motor_ros/config/four_can_real_robot.yaml` | CAN 口、电机 ID、关节名、话题、限幅、退出回零 |
| `src/dog_position_control/config/gait_controller_real_stand.yaml` | 真机站立、步态、`/cmd_vel`、`/gait_action` 参数 |
| `src/dog_position_control/config/gait_controller_sim_trot.yaml` | 仿真步态参数 |
| `src/dog_teleop/config/dog_teleop.yaml` | 手柄轴/按键映射和速度上限 |
| `src/arm_control/config/box_motion.yaml` | 机械臂箱子吸取、抬起、放置动作参数 |
| `src/suction_controller/config/suction_controller.yaml` | 吸盘/放气阀继电器串口和控制字节 |
| `src/dog_bringup/config/robot_controllers.yaml` | Gazebo ros2_control 控制器 |
| `src/dog_bringup/config/99-serial-can.rules` | 真机设备稳定命名 |

## 当前开发重点

任务赛完整闭环还缺这些关键模块：

- 智力题视觉识别、四则运算求解、答案对 4 取模、高分区发布。
- 根据箱体标签识别结果自动选择食品、工具、仪器/建材、药品的搬运顺序。
- 归位区坐标建图、从物资箱到归位区的自主导航。
- 底盘到位、机械臂抓取、带箱移动、释放、下一箱循环的任务级状态机。
- 摔倒/故障后的合规恢复流程、计时和任务日志。
- IMU 姿态闭环、抓取成功检测、箱体放置成功检测。

更详细的状态请看 [任务赛代码完成度与待办](docs/task_implementation_status.md)。

## 提交说明

`build/`、`install/`、`log/` 是本地生成目录，默认不提交。规则 PDF 是当前文档依据，后续如果官方发布新版本，需要重新更新 `docs/task_competition_rules.md`。
