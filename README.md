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

### 5. 常用调试命令

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
