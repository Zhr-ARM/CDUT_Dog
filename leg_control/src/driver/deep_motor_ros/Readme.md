# DeepRobotics J60 电机驱动节点 (ROS 2)

该功能包是绝影 J60 系列关节电机的 **ROS 2 硬件接口层 (Hardware Interface)**。它负责通过 SocketCAN 与底层电机通信，并向算法层提供标准的 ROS 话题接口。

## 📋 功能特性

* **多电机级联控制**：支持通过参数配置同时控制任意数量的电机（默认 ID: 1, 2, 3）。
* **智能连接检测**：启动时自动握手，自动剔除离线或无响应的电机，防止报错刷屏。
* **严格 ID 验证**：防止 CAN 总线缓存数据导致的电机 ID 误报。
* **硬件自动配置**：节点启动时自动申请 `sudo` 权限配置 CAN 波特率（1Mbps）和队列长度。

---

## 🛠️ 编译与安装

### 1. 依赖项

* Ubuntu 22.04 + ROS 2 Humble
* 支持 SocketCAN 的 USB-CAN 模块（如 candleLight, 创芯科技等，需刷写 Linux 固件）

### 2. 编译

请在工作空间根目录下执行：

```bash
# 编译整个工作空间
colcon build --symlink-install

# 或者仅编译驱动包
colcon build --packages-select deep_motor_ros

```

### 3. **⚠️ 重要配置 (必读)**

由于代码中包含了自动配置 CAN 接口的功能（自动执行 `ip link set up...`），**请务必检查以下代码中的 sudo 密码配置**：

打开 `src/driver/deep_motor_ros/src/motor_node.cpp`，找到 `main` 函数：

```cpp
// 确保这里的密码是您当前工控机/虚拟机的登录密码
std::string sudo_password = "llq666"; 

```

*如果密码不正确，CAN 接口将无法自动启动。*

---

## 🚀 快速启动

### 1. 运行节点

```bash
source install/setup.bash
ros2 run deep_motor_ros motor_node

```

*正常启动日志示例：*

```text
[INFO] ... 正在检测电机 ID: 1 ...
[INFO] >>> 电机 (ID: 1) 连接成功！
[INFO] ... 正在检测电机 ID: 2 ...
[WARN] >>> 电机 (ID: 2) 未连接。
[INFO] 初始化完成。共检测到 1/3 个电机在线。

```

### 2. 修改电机 ID (运行时参数)

如果电机id分别为11，13，15可以这样启动：

```bash
ros2 run deep_motor_ros motor_node --ros-args -p motor_ids:="[11, 13, 15]"

```

---

## 📡 接口文档 (算法对接)

### 1. 控制指令 (Input)

* **话题**: `/motor_cmd`
* **类型**: `std_msgs/msg/Float64MultiArray`
* **频率**: 建议 100Hz ~ 500Hz

**数据协议：**
发送一个**一维数组**，数组长度必须严格等于 `电机数量 × 5`。数据按电机顺序平铺。

| 索引 (Index) | 参数 | 单位 | 说明 | 推荐范围 |
| --- | --- | --- | --- | --- |
| **0** | **Pos** | `rad` | 期望位置 | - |
| **1** | **Vel** | `rad/s` | 期望速度 | - |
| **2** | **Torque** | `N·m` | 前馈力矩 | - |
| **3** | **Kp** | - | 位置刚度 | 0 ~ 100 |
| **4** | **Kd** | - | 速度阻尼 | 0 ~ 5 |
| **5** | Pos (Motor 2) | ... | 下一个电机的参数 | ... |
| ... | ... | ... | ... | ... |

**Python 发送示例 (3个电机)：**

```python
# M1: 位置控制, M2: 阻尼模式, M3: 零力矩
data = [
    3.14, 0.0, 0.0, 20.0, 1.0,  # Motor 1
    0.0,  0.0, 0.0, 0.0,  2.0,  # Motor 2
    0.0,  0.0, 0.0, 0.0,  0.0   # Motor 3
]

```

---

### 2. 状态反馈 (Output)

* **话题**: `/motor_feedback`
* **类型**: `std_msgs/msg/Float64MultiArray`
* **频率**: 20Hz (默认)

**数据协议：**
接收到的也是一个**一维数组**，长度为 `电机数量 × 5`。

| 索引 (Index) | 参数 | 单位 | 说明 |
| --- | --- | --- | --- |
| **0** | **Pos** | `rad` | 实际角度 |
| **1** | **Vel** | `rad/s` | 实际角速度 |
| **2** | **Torque** | `N·m` | 实际反馈力矩 |
| **3** | **T_Motor** | `℃` | 电机线圈温度 |
| **4** | **T_Driver** | `℃` | 驱动板温度 |
| **5** | Pos (Motor 2) | ... | 下一个电机的反馈 |

---

### 3. 可视化 (Rviz)

* **话题**: `/joint_states`
* **类型**: `sensor_msgs/msg/JointState`
* **说明**: 该话题仅用于 Rviz 显示机器人姿态，包含 `name`, `position`, `velocity`, `effort`。不包含温度信息。

---

## 🧪 测试命令

**1. 查看反馈数据**

```bash
ros2 topic echo /motor_feedback

```

*如果数据不更新，请检查 CAN 线的连接以及终端电阻是否安装。*

**2. 发送测试指令 (小心电机转动!)**
让 ID 1 的电机以较小的刚度转动到 1.57 rad (90度)：
*(假设当前配置了3个电机，需要补齐后两个电机的 0)*

```bash
ros2 topic pub /motor_cmd std_msgs/msg/Float64MultiArray "{data: [1.57, 0.0, 0.0, 5.0, 0.5,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0]}" --once

```

---

$$