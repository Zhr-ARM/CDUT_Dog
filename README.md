# CDUT_Dog

这是一个基于 ROS 2 的四足机器人工作空间，包含机器人模型与启动、MIT 风格关节控制器、步态控制节点，以及真机电机驱动接口。

当前仓库以 `colcon` 工作空间组织，源码主要位于 `src/` 下。

## 文件架构

```text
CDUT_Dog/
├── src/
│   ├── deep_motor_ros/          # 真机电机驱动，负责 CAN 通信与状态反馈
│   ├── dog_bringup/             # 机器人模型、消息定义、仿真/真机启动文件
│   ├── dog_position_control/    # 四足步态控制节点与参数
│   ├── dog_imu/                 # WitMotion IMU 驱动，发布姿态/角速度/加速度
│   ├── dog_teleop/              # Xbox/PS4 手柄遥控节点，将摇杆输入转为速度指令
│   └── mit_controller/          # ros2_control 控制器插件
├── build/                       # colcon 编译产物
├── install/                     # colcon 安装产物
├── log/                         # colcon 日志
└── README.md
```

### 各功能包说明

#### `src/dog_bringup`

- `launch/`
  - `sim.launch.py`：启动 Gazebo、机器人模型、`mit_controller` 和步态控制器
  - `real.launch.py`：启动真机电机驱动，并可选同时带起仿真
  - `hardware_gait.launch.py`：对 `real.launch.py` 的简单封装
- `config/`
  - `dog.world`：Gazebo 世界
  - `dog.rviz`：RViz 配置
  - `robot_controllers.yaml`：`controller_manager` 与 `mit_controller` 参数
  - `description/`：导出的 URDF 与参考配置
- `xacro/cdut_dog/`：机器人模型 xacro/urdf
- `meshes/cdut_dog/`：机器人网格模型
- `msg/`：自定义消息

#### `src/dog_position_control`

- `src/quadruped_gait_controller.cpp`：四足步态控制主节点
- `launch/gait_controller.launch.py`：单独启动步态控制器
- `config/gait_controller_sim_trot.yaml`：小跑步态参数、站立参数、关节增益等核心配置

#### `src/mit_controller`

- `src/mit_controller.cpp`：MIT 风格关节控制器实现
- `include/mit_controller/`：控制器头文件
- `mit_controller_plugin.xml`：插件导出描述

#### `src/dog_teleop`

- `src/dog_teleop_node.cpp`：手柄遥控主节点，订阅 `/joy`，发布 `/cmd_vel`
- `launch/dog_teleop.launch.py`：同时启动 `joy_node`（读取手柄设备）和 `dog_teleop_node`
- `config/dog_teleop.yaml`：最大线速度、最大角速度、死区阈值、超时时间等参数

摇杆映射关系（仅使用左摇杆）：

| 输入 | 话题/字段 | 说明 |
|---|---|---|
| 左摇杆 Y 轴（axes[1]） | `cmd_vel.linear.x` | 上推前进（+）/ 下拉后退（−） |
| 左摇杆 X 轴（axes[0]） | `cmd_vel.angular.z` | 左推左转（+）/ 右推右转（−） |
| A 键（buttons[0]） | — | 立即发布零速（急停） |

左摇杆两轴共同决定运动方向与速度：推向左前方 = 边走边左转，幅度越大速度越快。对角方向自动归一化，合速度不会超过最大值。

`/cmd_vel` 由 `dog_position_control` 中的步态控制器订阅，控制实际行走速度与转向。

#### `src/dog_imu`

WitMotion IMU（型号 IWT603）的 ROS 2 驱动包，支持多种通信协议。

- `dog_imu/wit_normal_node.py`：标准串口协议驱动节点
- `dog_imu/wit_modbus_node.py`：Modbus 协议驱动节点
- `dog_imu/wit_can_node.py`：CAN 协议驱动节点
- `dog_imu/get_imu_rpy.py`：订阅 IMU 数据并打印 Roll/Pitch/Yaw 的调试工具
- `launch/wit_imu.launch.py`：IMU 驱动启动文件，通过 `type` 参数选择协议

发布的话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/wit/imu` | `sensor_msgs/Imu` | 四元数姿态、角速度、线加速度 |
| `/wit/mag` | `sensor_msgs/MagneticField` | 磁力计数据 |
| `/wit/location` | `sensor_msgs/NavSatFix` | GPS 经纬度与海拔 |

> 当前 IMU 驱动已可独立运行，尚未接入步态控制闭环。后续计划用 IMU 的俯仰角反馈替代步态控制器中基于正运动学的间接姿态估算，实现机身俯仰闭环稳定。

#### `src/deep_motor_ros`

- `src/motor_node.cpp`：真机电机驱动节点
- `config/four_can_real_robot.yaml`：CAN 接口、电机 ID、关节名、话题映射等配置
- `launch/motor_bringup.launch.py`：电机驱动启动入口

## 使用方法

### 1. 编译工作空间

在工作空间根目录执行：

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果你使用的是 ROS 2 Humble，可将 `<ros_distro>` 替换为 `humble`。

### 2. 启动 Gazebo 仿真

```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_bringup sim.launch.py
```

默认行为：

- 启动 Gazebo
- 加载机器人模型
- 启动 `joint_state_broadcaster`
- 启动 `mit_controller`
- 启动 `quadruped_gait_controller`
- 启动 RViz

如果只想关闭 RViz：

```bash
ros2 launch dog_bringup sim.launch.py launch_rviz:=false
```

如果想替换步态参数文件：

```bash
ros2 launch dog_bringup sim.launch.py \
  gait_param_file:=/home/zhr/robot_dog/src/dog_position_control/config/gait_controller_sim_trot.yaml
```

### 3. 单独启动步态控制器

当底层关节状态和命令通道已经准备好时，可以单独启动步态节点：

```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_position_control gait_controller.launch.py
```

### 4. 启动真机控制

真机模式会启动电机驱动节点和四足步态控制节点：

```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_bringup real.launch.py
```

或者：

```bash
ros2 launch dog_bringup hardware_gait.launch.py
```

如果只想启动真机链路，不同时启动 Gazebo，可使用：

```bash
ros2 launch dog_bringup real.launch.py launch_simulation:=false
```

真机启动前建议优先检查：

- `src/deep_motor_ros/config/four_can_real_robot.yaml`
  - `can_interfaces`
  - `motors_per_can`
  - `motor_ids`
  - `joint_names`
  - `command_topic`
  - `joint_state_topic`
- `src/dog_position_control/config/gait_controller_sim_trot.yaml`
  - `use_sim_time`
  - `command_topic`
  - 站立姿态参数与步态参数

### 5. 手柄遥控

手柄遥控需要在仿真或真机已启动的基础上，**额外**启动 `dog_teleop`。

#### 5.1 连接手柄并启动遥控节点

将 Xbox 或 PS4 手柄通过 USB / 蓝牙连接到主机，然后：

```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_teleop dog_teleop.launch.py
```

该命令会同时启动：
- `joy_node`：读取 `/dev/input/js0`（或自动检测手柄设备）
- `dog_teleop_node`：将摇杆值映射为 `/cmd_vel`

#### 5.2 仿真 + 手柄完整启动流程

打开两个终端：

**终端 1（仿真）：**
```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_bringup sim.launch.py
```

**终端 2（手柄）：**
```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_teleop dog_teleop.launch.py
```

#### 5.3 手柄操作说明

所有运动控制集中在左摇杆，方向与速度由摇杆二维向量决定：

| 操作 | 效果 |
|---|---|
| 左摇杆向前推 | 机器狗前进，推幅越大速度越快 |
| 左摇杆向后拉 | 机器狗后退 |
| 左摇杆向左推 | 原地左转（CCW） |
| 左摇杆向右推 | 原地右转（CW） |
| 左摇杆推向对角 | 同时前进/后退 + 转弯，合速度自动归一化不超速 |
| 摇杆归中 | 速度归零，机器狗自动切换到站立状态 |
| A 键 | 立即急停，发布零速指令 |

> **注意：** 右摇杆无功能。步态控制器收到非零 `/cmd_vel` 时自动切入行走状态；摇杆归中或手柄超过 0.5 秒未发送数据时自动切回站立。

#### 5.4 不接手柄时用命令行验证

```bash
# 前进
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 持续发布（模拟手柄保持前进，Ctrl+C 停止后机器狗自动站立）
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 5.5 相关参数调整

`src/dog_position_control/config/gait_controller_sim_trot.yaml` 中与遥控相关的参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel` | 订阅的速度指令话题 |
| `cmd_vel_timeout` | `0.5` s | 超时后自动归零 |
| `max_linear_vel` | `0.5` m/s | 对应满幅步长的线速度上限 |
| `max_angular_vel` | `1.0` rad/s | 对应满幅差速的角速度上限 |
| `cmd_vel_deadzone` | `0.05` | 速度幅值低于此值视为停止 |
| `foot_x_signs` | `[1,1,1,1]` | 各腿前进方向符号；若前进方向反了改为 `[-1,-1,-1,-1]` |

### 6. 启动 IMU

将 WitMotion IMU 通过 USB 连接后：

```bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
ros2 launch dog_imu wit_imu.launch.py type:=normal port:=/dev/ttyUSB0 baud:=9600
```

`type` 可选 `normal`（默认）、`modbus`、`hmodbus`、`can`、`hcan`，根据 IMU 固件协议选择。

验证 IMU 数据：

```bash
# 查看原始 IMU 消息
ros2 topic echo /wit/imu

# 查看 Roll/Pitch/Yaw（角度制）
ros2 run dog_imu get_imu_rpy
```

### 7. 常用调试命令

查看关节状态：

```bash
ros2 topic echo /joint_states
```

查看真机反馈：

```bash
ros2 topic echo /motor_feedback
```

查看控制命令：

```bash
ros2 topic echo /motor_cmd
```

查看手柄原始输入：

```bash
ros2 topic echo /joy
```

查看遥控速度指令：

```bash
ros2 topic echo /cmd_vel
```

确认步态控制器已订阅速度指令：

```bash
ros2 topic info /cmd_vel
```

查看 IMU 姿态数据：

```bash
ros2 topic echo /wit/imu
```

查看 IMU Roll/Pitch/Yaw：

```bash
ros2 run dog_imu get_imu_rpy
```

## 关键配置说明

### `src/dog_position_control/config/gait_controller_sim_trot.yaml`

这是当前步态控制的核心参数文件，主要负责：

- 步态类型与节拍
- 抬腿高度、步长、周期
- 启动站立与站起流程
- 关节零位偏置
- HAA / HFE / KFE 的 PD 增益
- 前后腿摆动补偿与姿态平衡补偿

修改这个文件后，通常重新启动 `sim.launch.py` 或 `gait_controller.launch.py` 即可生效。

### `src/deep_motor_ros/config/four_can_real_robot.yaml`

这个文件决定真机通信拓扑，包括：

- 使用哪些 CAN 口
- 每条 CAN 上挂载多少电机
- 电机 ID 与关节名映射
- 指令与反馈话题名称
- 控制频率与日志周期

### `src/dog_bringup/config/robot_controllers.yaml`

这个文件用于配置 `controller_manager` 下的控制器，主要包括：

- `mit_controller` 插件类型
- 12 个关节名称
- 控制模式
- 指令话题与反馈话题
- 力矩限幅

## 说明

- 仓库中的 `build/`、`install/`、`log/` 为本地生成目录，默认不提交。
- 如果后续要补充更详细的安装依赖、硬件接线、参数调参说明，可以继续在此 README 基础上扩展。
