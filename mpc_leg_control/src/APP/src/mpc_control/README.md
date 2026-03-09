# mpc_control — 轻量级步态控制器

> ROS2 步态控制功能包，支持 `walk` / `run` / `jump` 三种步态模式，面向 `leg_mit_controller` 仿真接口。

---

## 📋 概述

`mpc_control` 实现了一个**足端轨迹规划 + 逆运动学 + PD控制 + 重力前馈**的步态控制方案，输出对接 `leg_mit_controller` 的 `/motor_cmd`（15维 MIT 格式命令）。

### 控制流水线

```
步态状态机 → 足端轨迹生成 → 逆运动学(IK) → 雅可比速度映射 → PD+前馈力矩 → 发布 /motor_cmd
(摆动/支撑)    (摆线/线性)     (足端→关节角)   (足端速度→角速度)  (重力补偿)      (15维MIT命令)
```

### 功能特点

- ✅ 三种步态模式实时切换（walk / run / jump）
- ✅ 摆线（cycloid）平滑摆动相轨迹
- ✅ 解析雅可比矩阵实现关节速度前馈
- ✅ 简化重力补偿前馈力矩
- ✅ 一阶低通滤波平滑关节命令
- ✅ 自动零位偏置计算

---

## 📁 目录结构

```text
mpc_control/
├── CMakeLists.txt                  # CMake 构建配置
├── package.xml                     # ROS2 包描述
├── README.md                       # 本文件
├── config/
│   └── mpc_gait_controller.yaml    # 参数配置文件（步态、PD增益、物理参数等）
├── launch/
│   └── mpc_gait_sim.launch.py      # 仿真启动脚本
├── src/
│   └── mpc_gait_controller.cpp     # 核心控制器源码（含详细中文注释）
└── docs/
    └── mpc_control_guide.md        # 详细技术文档（算法原理、调参指南等）
```

---

## 🔌 ROS2 接口

| 方向 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **发布** | `/motor_cmd` | `std_msgs/Float64MultiArray` | 15维 MIT 命令（3关节×5参数） |
| **订阅** | `/gait_mode` | `std_msgs/String` | 步态模式切换（`walk`/`run`/`jump`） |

### /motor_cmd 消息格式（15维）

```text
[0~4]   髋关节:     目标角度, 目标角速度, 前馈力矩, Kp, Kd
[5~9]   大腿关节:   目标角度, 目标角速度, 前馈力矩, Kp, Kd
[10~14] 膝曲柄关节: 目标角度, 目标角速度, 前馈力矩, Kp, Kd
```

电机端执行的力矩公式：`τ = Kp·(q_d - q) + Kd·(qd_d - qd) + τ_ff`

---

## 🚀 快速使用

### 1. 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/llq/mpc_leg_control
colcon build --packages-select mpc_control
source install/setup.bash
```

### 2. 启动仿真 + 控制器

```bash
# 终端1: 启动 Gazebo 仿真
ros2 launch leg_model gazebo.launch.py

# 终端2: 启动步态控制器
ros2 launch mpc_control mpc_gait_sim.launch.py
```

### 3. 切换步态

```bash
# 切换到 walk 步态
ros2 topic pub /gait_mode std_msgs/msg/String "{data: walk}" -1

# 切换到 run 步态
ros2 topic pub /gait_mode std_msgs/msg/String "{data: run}" -1

# 切换到 jump 步态
ros2 topic pub /gait_mode std_msgs/msg/String "{data: jump}" -1
```

---

## ⚙️ 主要参数

参数文件位于 `config/mpc_gait_controller.yaml`，可通过 launch 命令覆盖：

```bash
ros2 launch mpc_control mpc_gait_sim.launch.py param_file:=/path/to/your.yaml
```

### 机器人物理参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `link_length_thigh` | 0.200 | 大腿连杆长度 [m] |
| `link_length_shank` | 0.243 | 小腿连杆长度 [m] |
| `leg_mass` | 1.05 | 腿部等效质量 [kg] |
| `leg_com_ratio` | 0.45 | 质心在大腿上的位置比例 |
| `enable_knee_gravity_comp` | true | 是否启用膝关节重力补偿 |

### 步态参数

| 参数 | walk | run | jump | 说明 |
|------|------|-----|------|------|
| `step_length` | 0.08 | 0.12 | 0.00 | 步长 [m] |
| `step_height` | 0.05 | 0.06 | 0.12 | 步高 [m] |
| `period` | 0.80 | 0.50 | 0.65 | 步态周期 [s] |
| `swing_ratio` | 0.60 | 0.60 | 0.45 | 摆动相占比 |
| `stance_depth` | 0.29 | 0.28 | 0.30 | 站立深度 [m] |

### PD 控制增益

| 关节 | Kp | Kd |
|------|----|----|
| 髋关节 (hip) | 3.0 | 0.8 |
| 大腿 (thigh) | 12.0 | 4.0 |
| 膝曲柄 (knee_crank) | 12.0 | 4.0 |

### 其他参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `control_rate` | 200.0 | 控制频率 [Hz] |
| `filter_alpha` | 0.25 | 低通滤波系数（越小越平滑） |
| `auto_zero_offsets` | true | 自动计算零位偏置 |

---

## 🔧 调参建议

1. **先稳再快**：先用 walk 步态调稳，降低 `step_length`、增大 `period`
2. **抖动处理**：减小 `filter_alpha`（更平滑）或降低 `Kp`
3. **关节振荡**：降低 `thigh_kp` / `knee_crank_kp`，或提高对应 `Kd`
4. **跳跃高度**：调整 `jump.step_height` 与 `jump.period`
5. **站立姿态**：修改 `stance_depth` 控制腿部伸展程度

---

## 📖 详细文档

更完整的算法原理、架构设计和技术细节，请参阅 [docs/mpc_control_guide.md](docs/mpc_control_guide.md)。
